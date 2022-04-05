/* Copyright (c) Codethink Ltd. All rights reserved.
   Licensed under the MIT License. */

#include "SPIMasterLow.h"
#include "Common.h"
#include "NVIC.h"
#include "mt3620/spi.h"
#include "mt3620/dma.h"

typedef enum {
    SPI_MASTER_TRANSFER_WRITE,
    SPI_MASTER_TRANSFER_READ,
    SPI_MASTER_TRANSFER_FULL_DUPLEX,
} SPIMaster_TransferType;

struct SPIMaster {
    uint32_t      id;
    bool          open;
    bool          dma;
    SPI_IdleLevel idleLevel;
    uint32_t      csLine;
    void        (*csCallback)(SPIMaster*, bool);
    bool          csEnable;
    void        (*callback)(int32_t, uintptr_t);
    void        (*callbackUser)(int32_t, uintptr_t, void*);
    void         *userData;
    SPITransfer  *xfer;
    unsigned      xferCount, xferDone;
    int32_t       dataCount;
};

static SPIMaster spiContext[MT3620_SPI_COUNT] = { 0 };

// Note that we currently reserve a buffer in sysram for each possible ISU interface
// for very sysram constrained applications it may make sense to modify this so that
// you only reserve the buffers that are actually needed.
static __attribute__((section(".sysram"))) mt3620_spi_dma_cfg_t SPIMaster_DmaConfig[MT3620_SPI_COUNT] = { 0 };

#define SPI_PRIORITY 2

int32_t SPIMaster_SelectIdleLineLevel(SPIMaster *handle, SPI_IdleLevel level)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }
    if (!handle->open) {
        return ERROR_HANDLE_CLOSED;
    }

    switch (level) {
    case SPI_IDLE_LEVEL_LOW:
    case SPI_IDLE_LEVEL_HIGH:
    case SPI_IDLE_LEVEL_DONT_CARE:
        break;
    default:
        return ERROR_UNSUPPORTED;
    }

    if (handle->xferCount > handle->xferDone) {
        return ERROR_BUSY;
    }

    handle->idleLevel = level;
    return ERROR_NONE;
}

int32_t SPIMaster_Select(SPIMaster *handle, unsigned csLine)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }
    if (!handle->open) {
        return ERROR_HANDLE_CLOSED;
    }

    if (csLine > MT3620_CS_MAX) {
        return ERROR_UNSUPPORTED;
    }

    handle->csLine     = csLine;
    handle->csEnable   = true;
    handle->csCallback = NULL;

    // Set the chip select line.
    SPIMaster_DmaConfig[handle->id].smmr.rs_slave_sel = csLine;

    return ERROR_NONE;
}

int32_t SPIMaster_SelectEnable(SPIMaster *handle, bool enable)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }
    if (!handle->open) {
        return ERROR_HANDLE_CLOSED;
    }

    handle->csEnable = enable;
    if (!handle->csCallback) {
        SPIMaster_DmaConfig[handle->id].smmr.rs_slave_sel = enable ? handle->csLine : MT3620_CS_NULL;
    }

    return ERROR_NONE;
}

int32_t SPIMaster_SetSelectLineCallback(
    SPIMaster *handle,
    void       (*csCallback)(SPIMaster *handle, bool select))
{
    if (!handle) {
        return ERROR_PARAMETER;
    }

    handle->csCallback = csCallback;
    handle->csEnable   = (csCallback != NULL);
    handle->csLine     = MT3620_CS_NULL;

    SPIMaster_DmaConfig[handle->id].smmr.rs_slave_sel = MT3620_CS_NULL;

    return ERROR_NONE;
}

int32_t SPIMaster_Configure(SPIMaster *handle, bool cpol, bool cpha, uint32_t busSpeed)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }
    if (!handle->open) {
        return ERROR_HANDLE_CLOSED;
    }

    // There's an errata for low busSpeed values when CPOL and CPHA are 0, so we increase the minimum.
    if ((cpol == 0) && (cpha == 0) && (busSpeed < 250000)) {
        return ERROR_UNSUPPORTED;
    }

    // We round up the clock-speed division here to get the closest speed below the target.
    unsigned rs_clk_sel = ((MT3620_SPI_HCLK + (busSpeed - 1)) / busSpeed);
    rs_clk_sel = (rs_clk_sel < 2 ? 0 : (rs_clk_sel - 2));

    // Check we're not below the minimum speed.
    if (rs_clk_sel > 4095) {
        return ERROR_UNSUPPORTED;
    }

    mt3620_spi_dma_cfg_t *cfg = &SPIMaster_DmaConfig[handle->id];
    cfg->smmr.cpol          = cpol;       // Set polarity for CPOL setting.
    cfg->smmr.cpha          = cpha;       // Set polarity for CPHA setting.
    cfg->smmr.rs_clk_sel    = rs_clk_sel; // Set serial clock SPI_CLK.
    cfg->smmr.more_buf_mode = 1;          // Select SPI buffer size.
    cfg->smmr.lsb_first     = false;      // Select MSB first.
    cfg->smmr.int_en        = true;       // Enable interrupts.

    return ERROR_NONE;
}

int32_t SPIMaster_DMAEnable(SPIMaster *handle, bool enable)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }
    if (!handle->open) {
        return ERROR_HANDLE_CLOSED;
    }

    if (handle->dma == enable) {
        return ERROR_NONE;
    }

    if (MT3620_DMA_FIELD_READ(MT3620_SPI_DMA_TX(handle->id), start, str)) {
        return ERROR_BUSY;
    }

    MT3620_SPI_FIELD_WRITE(handle->id, cspol, dma_mode, enable);
    handle->dma = enable;
    return ERROR_NONE;
}

int32_t SPIMaster_ConfigureDriveStrength(SPIMaster *handle, unsigned drive)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }
    if (!handle->open) {
        return ERROR_HANDLE_CLOSED;
    }

    if (drive == 0) {
        return ERROR_PARAMETER;
    }

    if (drive < 4) {
        return ERROR_UNSUPPORTED;
    }

    drive = ((drive - 4) >> 2);
    if (drive > 3) {
        drive = 3;
    }

    // Note that at the time of writing this register address is not documented
    // in the functional spec.
    volatile uint32_t *paddrv = (uint32_t *)((0x38070000 + (0x00010000 * handle->id)) | 0x0070);

    uint32_t mask = *paddrv;
    mask |= (drive << 0); // SCK
    mask |= (drive << 2); // MOSI
    mask |= (drive << 4); // MISO
    mask |= (drive << 6); // CSA
    mask |= (drive << 8); // CSB
    *paddrv = mask;

    return ERROR_NONE;
}


static inline unsigned SPIMaster_UnitToID(Platform_Unit unit)
{
    if ((unit < MT3620_UNIT_ISU0) || (unit > MT3620_UNIT_ISU5)) {
        return MT3620_SPI_COUNT;
    }
    return (unit - MT3620_UNIT_ISU0);
}

SPIMaster *SPIMaster_Open(Platform_Unit unit)
{
    unsigned id = SPIMaster_UnitToID(unit);
    if ((id >= MT3620_SPI_COUNT) || (spiContext[id].open)) {
        return NULL;
    }

    spiContext[id].id           = id;
    spiContext[id].open         = true;
    spiContext[id].dma          = true;
    spiContext[id].idleLevel    = SPI_IDLE_LEVEL_HIGH;
    spiContext[id].csLine       = MT3620_CS_NULL;
    spiContext[id].csCallback   = NULL;
    spiContext[id].csEnable     = false;
    spiContext[id].callback     = NULL;
    spiContext[id].callbackUser = NULL;
    spiContext[id].userData     = NULL;
    spiContext[id].xfer         = NULL;
    spiContext[id].xferCount    = 0;
    spiContext[id].xferDone     = 0;
    spiContext[id].dataCount    = 0;

    // Select the CS line.
    int32_t status = SPIMaster_Select(&spiContext[id], 0);
    if (status != ERROR_NONE) {
        return NULL;
    }

    // Call SPIMaster_Configure to set up the chip for the SPI to 2 MHz by default.
    status = SPIMaster_Configure(&spiContext[id], 0, 0, 2000000);
    if (status != ERROR_NONE) {
        return NULL;
    }

    // Enable and Set the NVIC interrupt priority.
    NVIC_EnableIRQ(MT3620_SPI_INTERRUPT(id), SPI_PRIORITY);

    // Hard-code start to true in DMA transfers so transfer starts.
    mt3620_spi_dma_cfg_t *cfg = &SPIMaster_DmaConfig[id];
    cfg->stcsr.spi_master_start = true;

    volatile mt3620_dma_t * const tx_dma = &mt3620_dma[MT3620_SPI_DMA_TX(id)];
    mt3620_dma_global->ch_en_set = (1U << MT3620_SPI_DMA_TX(id));
    MT3620_DMA_FIELD_WRITE(MT3620_SPI_DMA_TX(id), start, str, false);
    mt3620_dma_con_t dma_con_tx = { .mask = tx_dma->con };
    dma_con_tx.dir   = 0;
    dma_con_tx.wpen  = false;
    dma_con_tx.wpsd  = 0;
    dma_con_tx.iten  = false;
    dma_con_tx.hiten = false;
    dma_con_tx.dreq  = false;
    dma_con_tx.dinc  = 0;
    dma_con_tx.sinc  = 1;
    dma_con_tx.size  = 2;
    tx_dma->con = dma_con_tx.mask;
    tx_dma->fixaddr = (uint32_t *)&mt3620_spi[id]->dataport;
    tx_dma->pgmaddr = (uint32_t *)&SPIMaster_DmaConfig[id];

    // Enable DMA mode.
    MT3620_SPI_FIELD_WRITE(id, cspol, dma_mode, true);

    // We have to set all buffers to know the the MOSI line idle level.
    // This is due to a hardware bug in the SPI adapter on the MT3620.
    if (spiContext[id].idleLevel != SPI_IDLE_LEVEL_DONT_CARE) {
        uint32_t fill = (spiContext[id].idleLevel == SPI_IDLE_LEVEL_LOW
            ? 0x00000000 : 0xFFFFFFFF);

        unsigned i;
        for (i = 0; i < (MT3620_SPI_BUFFER_SIZE_HALF_DUPLEX / 4); i++) {
            mt3620_spi[id]->sdor[i] = fill;
        }
        mt3620_spi[id]->soar = fill;
    }

    MT3620_SPI_FIELD_WRITE(id, stcsr, spi_master_start, false);

    // Clear interrupt flag
    (void)mt3620_spi[id]->scsr;

    return &spiContext[id];
}

void SPIMaster_Close(SPIMaster *handle)
{
    if (!handle || !handle->open) {
        return;
    }

    mt3620_dma_global->ch_en_clr = (1U << MT3620_SPI_DMA_TX(handle->id));
    MT3620_SPI_FIELD_WRITE(handle->id, stcsr, spi_master_start, false); // Stop transfers
    MT3620_SPI_FIELD_WRITE(handle->id, smmr , int_en          , false); // Disable interrupts.
    MT3620_SPI_FIELD_WRITE(handle->id, cspol, dma_mode        , false); // Disable DMA mode.

    // Disable NVIC interrupts.
    NVIC_DisableIRQ(MT3620_SPI_INTERRUPT(handle->id));

    handle->open = false;
}


static inline void SPIMaster_WordCopy(volatile void *dst, volatile void *src, uintptr_t count)
{
    volatile uint32_t *udst = dst;
    volatile uint32_t *usrc = src;
    uintptr_t i;
    for (i = 0; i < count; i++) {
        udst[i] = usrc[i];
    }
}

static int32_t SPIMaster_TransferQueue(SPIMaster *handle, SPITransfer *xfer)
{
    mt3620_spi_dma_cfg_t *cfg = &SPIMaster_DmaConfig[handle->id];

    cfg->smmr.both_directional_data_mode = (xfer->readData && xfer->writeData);

    cfg->smbcr.mosi_bit_cnt = (xfer->writeData ? (xfer->length * 8) : 0);
    cfg->smbcr.miso_bit_cnt = (xfer->readData  ? (xfer->length * 8) : 0);
    cfg->smbcr.cmd_bit_cnt  = (xfer->opcodeLength  * 8);   

    uint8_t *sdor = (uint8_t *)cfg->sdor;
    if (xfer->writeData) {
        __builtin_memcpy(sdor, xfer->writeData, xfer->length);
    }

    // This workaround is required to make the MOSI line idle at known level due to SPI bug.
    if (handle->idleLevel != SPI_IDLE_LEVEL_DONT_CARE) {
        cfg->soar.mask = (handle->idleLevel == SPI_IDLE_LEVEL_LOW
            ? 0x00000000 : 0xFFFFFFFF);
        sdor[xfer->length % MT3620_SPI_BUFFER_SIZE_HALF_DUPLEX] =
            (handle->idleLevel == SPI_IDLE_LEVEL_LOW ? 0x00 : 0xFF);
    }

    if (xfer->opcodeLength > 0) {
        // Opcode bytes are sent big-endian so we need to endian-swap.
        const uint8_t *opcodeSrc = (const uint8_t *)xfer->opcodeData;
        uint8_t       *opcodeDst = (uint8_t *)&cfg->soar.mask;
        for (unsigned i = 0; i < xfer->opcodeLength; i++) {
              opcodeDst[xfer->opcodeLength - (i + 1)] = opcodeSrc[i];
        }
    }

    if (handle->dma) {
        unsigned index = MT3620_SPI_DMA_TX(handle->id);
        mt3620_dma[index].pgmaddr = (uint32_t*)&SPIMaster_DmaConfig[handle->id];
        mt3620_dma[index].count = (sizeof(mt3620_spi_dma_cfg_t) / 4);
        MT3620_DMA_FIELD_WRITE(index, start, str, true);
    } else {
        SPIMaster_WordCopy(&mt3620_spi[handle->id]->soar, &SPIMaster_DmaConfig[handle->id], 11);
        mt3620_spi[handle->id]->stcsr = SPIMaster_DmaConfig[handle->id].stcsr.mask;
    }

    return ERROR_NONE;
}

static bool SPIMaster_TransferValidate(
    SPIMaster *handle, const SPITransfer *transfer)
{
    if (!transfer->writeData && !transfer->readData) {
        return false;
    }

    if (transfer->writeData) {
        // All writes require an opcode.
        if ((transfer->opcodeLength == 0)
            || (transfer->opcodeLength > MT3620_SPI_OPCODE_SIZE)) {
            return false;
        }
    }

    if (transfer->readData) {
        // Both read and full-duplex require at least 1-byte in buffer.
        if (transfer->length ==  0) {
            return false;
        }
    }

    uint8_t limit = (transfer->readData && transfer->writeData
        ? MT3620_SPI_BUFFER_SIZE_FULL_DUPLEX
        : MT3620_SPI_BUFFER_SIZE_HALF_DUPLEX);

    uint8_t idleByte = (handle->idleLevel == SPI_IDLE_LEVEL_DONT_CARE ? 0 : 1);

    if (transfer->length > (limit - idleByte)) {
        return false;
    }

    return true;
}

static int32_t SPIMaster_TransferSequentialAsync_Core(
    SPIMaster   *handle,
    SPITransfer *transfer,
    uint32_t     count,
    void       (*callback)     (int32_t status, uintptr_t dataCount),
    void       (*callbackUser) (int32_t status, uintptr_t dataCount, void *userData),
    void        *userData)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }
    if (!handle->open) {
        return ERROR_HANDLE_CLOSED;
    }
    if (!transfer || (count == 0)) {
        return ERROR_PARAMETER;
    }

    if (handle->xferDone < handle->xferCount) {
        return ERROR_BUSY;
    }

    for (unsigned t = 0; t < count; t++) {
        if (!SPIMaster_TransferValidate(handle, &transfer[t])) {
            return ERROR_PARAMETER;
        }
    }

    handle->callback     = callback;
    handle->callbackUser = callbackUser;
    handle->userData     = userData;
    handle->xfer         = transfer;
    handle->xferCount    = count;
    handle->xferDone     = 0;
    handle->dataCount    = 0;

    if (handle->csEnable && handle->csCallback) {
        handle->csCallback(handle, true);
    }

    int32_t status = SPIMaster_TransferQueue(handle, &transfer[0]);
    if (status != ERROR_NONE) {
        handle->callback  = NULL;
        handle->xferCount = 0;
    }

    return status;
}

int32_t SPIMaster_TransferSequentialAsync(
    SPIMaster   *handle,
    SPITransfer *transfer,
    uint32_t     count,
    void        (*callback)(int32_t status, uintptr_t data_count))
{
    if (!handle) {
        return ERROR_PARAMETER;
    }

    if (handle->callback) {
        return ERROR_BUSY;
    }

    return SPIMaster_TransferSequentialAsync_Core(
        handle, transfer, count, callback, NULL, NULL);
}

int32_t SPIMaster_TransferSequentialAsync_UserData(
    SPIMaster   *handle,
    SPITransfer *transfer,
    uint32_t     count,
    void       (*callback)(int32_t status, uintptr_t data_count, void *userData),
    void        *userData)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }

    if (handle->callback) {
        return ERROR_BUSY;
    }

    return SPIMaster_TransferSequentialAsync_Core(
        handle, transfer, count, NULL, callback, userData);
}

int32_t SPIMaster_TransferCancel(SPIMaster *handle)
{
    if (!handle) {
        return ERROR_PARAMETER;
    }

    // Stop DMA, reset spi_master_start and read spi_scrc (to clear it)
    mt3620_spi_dma_cfg_t *cfg = &SPIMaster_DmaConfig[handle->id];
    MT3620_DMA_FIELD_WRITE(MT3620_SPI_DMA_TX(handle->id), start, str, false);
    cfg->stcsr.spi_master_start = false;
    uint32_t dummy_read = mt3620_spi[handle->id]->scsr;
    (void)dummy_read;

    handle->xferCount = 0;
    handle->xferDone  = 0;

    if (handle->callback) {
        handle->callback(ERROR_SPI_TRANSFER_CANCEL, 0);
        handle->callback = NULL;
    } else if (handle->callbackUser) {
        handle->callbackUser(ERROR_SPI_TRANSFER_CANCEL, 0, handle->userData);
        handle->callbackUser = NULL;
    }

    int32_t status = ERROR_NONE;
    if (handle->csEnable && handle->csCallback) {
        handle->csCallback(handle, false);
    }

    return status;
}

static volatile bool SPIMaster_TransferSequentialSync_Ready = false;
static int32_t       SPIMaster_TransferSequentialSync_Status;
static int32_t       SPIMaster_TransferSequentialSync_Count;

static void SPIMaster_TransferSequentialSync_Callback(int32_t status, uintptr_t data_count)
{
    SPIMaster_TransferSequentialSync_Status = status;
    SPIMaster_TransferSequentialSync_Count  = data_count;
    SPIMaster_TransferSequentialSync_Ready  = true;
}

int32_t SPIMaster_TransferSequentialSync(SPIMaster *handle, SPITransfer *transfer, uint32_t count)
{
    SPIMaster_TransferSequentialSync_Ready = false;
    int32_t status = SPIMaster_TransferSequentialAsync(
        handle, transfer, count, SPIMaster_TransferSequentialSync_Callback);
    if (status != ERROR_NONE) {
        return status;
    }

    while (!SPIMaster_TransferSequentialSync_Ready) {
        __asm__("wfi");
    }

    return SPIMaster_TransferSequentialSync_Status;
}

static void SPIMaster_IRQ(Platform_Unit unit)
{
    unsigned id = SPIMaster_UnitToID(unit);
    if (id >= MT3620_SPI_COUNT) {
        return;
    }

    SPIMaster *handle = &spiContext[id];

    // This should never happen
    if (!handle->open) {
        return;
    }

    if (handle->dma) {
        MT3620_DMA_FIELD_WRITE(MT3620_SPI_DMA_TX(id), start, str, false);
    }

    int32_t status = ERROR_NONE;

    // Clear interrupt flag and the status of the SPI transaction.
    if (!MT3620_SPI_FIELD_READ(id, scsr, spi_ok)) {
        status = ERROR_SPI_TRANSFER_FAIL;
    } else {
        if (handle->dma && (mt3620_dma[MT3620_SPI_DMA_TX(id)].rlct != 0)){
            status = ERROR_SPI_TRANSFER_FAIL;
        } else {
            SPITransfer *xfer = &handle->xfer[handle->xferDone];
            handle->dataCount += (xfer->opcodeLength + xfer->length);
        }
    }

    bool final = (status != ERROR_NONE);
    if (status == ERROR_NONE) {
        SPITransfer *transfer = &handle->xfer[handle->xferDone];

        if (transfer->readData) {
            uint8_t *sdir = (uint8_t*)mt3620_spi[handle->id]->sdir;
            __builtin_memcpy(transfer->readData, sdir, transfer->length);
        }

        handle->xferDone++;
        final = (handle->xferDone >= handle->xferCount);
        if (!final) {
            status = SPIMaster_TransferQueue(
                handle, &handle->xfer[handle->xferDone]);
            final = (status != ERROR_NONE);
        }
    }

    if (final) {
        if (handle->csEnable && handle->csCallback) {
            handle->csCallback(handle, false);
        }
        if (handle->callback) {
            handle->callback(status, handle->dataCount);
            handle->callback = NULL;
        } else if (handle->callbackUser) {
            handle->callbackUser(status, handle->dataCount, handle->userData);
            handle->callbackUser = NULL;
        }
    }
}

void isu_g0_spim_irq(void) { SPIMaster_IRQ(MT3620_UNIT_ISU0); }
void isu_g1_spim_irq(void) { SPIMaster_IRQ(MT3620_UNIT_ISU1); }
void isu_g2_spim_irq(void) { SPIMaster_IRQ(MT3620_UNIT_ISU2); }
void isu_g3_spim_irq(void) { SPIMaster_IRQ(MT3620_UNIT_ISU3); }
void isu_g4_spim_irq(void) { SPIMaster_IRQ(MT3620_UNIT_ISU4); }
void isu_g5_spim_irq(void) { SPIMaster_IRQ(MT3620_UNIT_ISU5); }
