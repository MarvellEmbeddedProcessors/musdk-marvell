/**
 * @file pp2_bm.c
 *
 * Buffer management and manipulation routines
 */
#include "pp2_types.h"

#include "pp2.h"
#include "pp2_bm.h"
#include "pp2_port.h"
#include "pp2_print.h"

/* TODO: temporary we add the prototypes here until we use the musdk ones */
uintptr_t cma_calloc(size_t size);
void cma_free(uintptr_t buf);
uintptr_t cma_get_vaddr(uintptr_t buf);
uintptr_t cma_get_paddr(uintptr_t buf);

#if PP2_BM_BUF_DEBUG
/* Debug and helpers */
static inline void pp2_bm_print_reg(uintptr_t cpu_slot,
        unsigned int reg_addr, const char *reg_name)
{
    pp2_dbg("  %-32s: 0x%X = 0x%08X\n", reg_name, reg_addr,
            pp2_reg_read(cpu_slot, reg_addr));
}

static void pp2_bm_pool_print_regs(uintptr_t cpu_slot, uint32_t pool)
{
    pp2_dbg("[BM pool registers: pool=%u]\n", pool);

    pp2_bm_print_reg(cpu_slot, MVPP2_BM_POOL_BASE_ADDR_REG(pool),
            "MVPP2_BM_POOL_BASE_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_POOL_SIZE_REG(pool),
            "MVPP2_BM_POOL_SIZE_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_POOL_READ_PTR_REG(pool),
            "MVPP2_BM_POOL_READ_PTR_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_POOL_PTRS_NUM_REG(pool),
            "MVPP2_BM_POOL_PTRS_NUM_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_BPPI_READ_PTR_REG(pool),
            "MVPP2_BM_BPPI_READ_PTR_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_BPPI_PTRS_NUM_REG(pool),
            "MVPP2_BM_BPPI_PTRS_NUM_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool),
            "MVPP2_BM_POOL_CTRL_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_INTR_CAUSE_REG(pool),
            "MVPP2_BM_INTR_CAUSE_REG");
    pp2_bm_print_reg(cpu_slot, MVPP2_BM_INTR_MASK_REG(pool),
            "MVPP2_BM_INTR_MASK_REG");
}
#endif
/** End debug helpers */

/* Set BM pool buffer size in HW.
 * Buffer size must be aligned to MVPP2_POOL_BUF_SIZE_OFFSET
 */
static inline void pp2_bm_pool_bufsize_set(uintptr_t cpu_slot,
        uint32_t pool_id, uint32_t buf_size)
{
    pp2_reg_write(cpu_slot, MVPP2_POOL_BUF_SIZE_REG(pool_id),
        buf_size << MVPP2_POOL_BUF_SIZE_OFFSET);
}

/* BM HW disable pool */
void pp2_bm_hw_pool_destroy(uintptr_t cpu_slot, uint32_t pool_id)
{
    uint32_t val;

    val = pp2_reg_read(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool_id));

    if (val & MVPP2_BM_STATE_MASK) {
        val |= MVPP2_BM_STOP_MASK;

        pp2_reg_write(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool_id), val);

        pp2_dbg("BM: stopping pool %u ...\n", pool_id);
        /* Wait pool stop notification */
        do {
            val = pp2_reg_read(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool_id));
        } while(val & MVPP2_BM_STATE_MASK);
        pp2_dbg("BM: stopped pool %u ...\n", pool_id);
    }

    /* Mask & Clear interrupt flags */
    pp2_reg_write(cpu_slot, MVPP2_BM_INTR_MASK_REG(pool_id), 0);
    pp2_reg_write(cpu_slot, MVPP2_BM_INTR_CAUSE_REG(pool_id), 0);

    /* Clear BPPE base */
    pp2_reg_write(cpu_slot, MVPP2_BM_POOL_BASE_ADDR_REG(pool_id), 0);
    pp2_reg_write(cpu_slot, MVPP22_BM_POOL_BASE_ADDR_HIGH_REG,
            0 & MVPP22_BM_POOL_BASE_ADDR_HIGH_MASK);
    /* Clear BPPE size */
    pp2_reg_write(cpu_slot, MVPP2_BM_POOL_SIZE_REG(pool_id),
            0 << BM_BPPESIZE_SHIFT);

    pp2_bm_pool_bufsize_set(cpu_slot, pool_id, 0);
#if PP2_BM_BUF_DEBUG
    pp2_bm_pool_print_regs(cpu_slot, pool_id);
#endif
}

/* BM pool hardware enable */
static uint32_t
pp2_bm_hw_pool_create(uintptr_t cpu_slot, uint32_t pool_id,
        uint32_t bppe_num, uintptr_t pool_phys_addr)
{
    uint32_t val;
    uint32_t phys_lo;
    uint32_t phys_hi;

    phys_lo = ((uint32_t)pool_phys_addr) & MVPP2_BM_POOL_BASE_ADDR_MASK;
    phys_hi = ((uint64_t)pool_phys_addr) >> 32;

    /* Check control register to see if this pool is already initialized */
    val = pp2_reg_read(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool_id));
    if (val & MVPP2_BM_STATE_MASK) {
        pp2_err("BM: pool=%u is already active\n", pool_id);
        return 1;
    }

    pp2_reg_write(cpu_slot, MVPP2_BM_POOL_BASE_ADDR_REG(pool_id), phys_lo);
    pp2_reg_write(cpu_slot, MVPP22_BM_POOL_BASE_ADDR_HIGH_REG,
            phys_hi & MVPP22_BM_POOL_BASE_ADDR_HIGH_MASK);
    pp2_reg_write(cpu_slot, MVPP2_BM_POOL_SIZE_REG(pool_id),
            (bppe_num / PP2_BPPE_UNIT_SIZE) << BM_BPPESIZE_SHIFT);

    val = pp2_reg_read(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool_id));
    val |= MVPP2_BM_START_MASK;

    pp2_reg_write(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool_id), val);

    /* Wait pool start notification */
    do {
        val = pp2_reg_read(cpu_slot, MVPP2_BM_POOL_CTRL_REG(pool_id));
    } while(! (val & MVPP2_BM_STATE_MASK));

    return 0;
}


int pp2_bm_pool_create(struct pp2* pp2, struct bm_pool_param *param)
{
    struct pp2_inst * pp2_inst;
    uintptr_t cpu_slot;
    uint32_t bppe_num;
    uint32_t bppe_size;
    uint32_t bppe_region_size;
    struct pp2_bm_pool *bm_pool;


    /* FS_A8K Table 1558: Provided buffer numbers divisible by
     * PP2_BPPE_UNIT_SIZE in order to avoid incomplete BPPEs */
    if (param->buf_num % PP2_BPPE_UNIT_SIZE) {
        pp2_err("BM: pool buffer number param must be a multiple of %u\n",
                PP2_BPPE_UNIT_SIZE);
        return -EACCES;
    }
    /* FS_A8K Table 1713: Buffer 32-byte aligned and greater
     * than packet offset configured in RXSWQCFG register
     */
    if (param->buf_size < PP2_PACKET_OFFSET) {
        pp2_err("BM: pool buffer size must be 32-byte aligned and"
                " greater than PP2_PACKET_OFFSET(%u)\n", PP2_PACKET_OFFSET);
        return -EACCES;
    }
    /* Allocate space for pool handler */
    bm_pool = calloc(1, sizeof(struct pp2_bm_pool));
    if (unlikely(!bm_pool)) {
        pp2_err("BM: cannot allocate memory for a BM pool\n");
        return -ENOMEM;
    }

    /* Offset pool hardware ID depending on configured pool offset */
    bm_pool->bm_pool_id = param->id;
    bm_pool->bm_pool_buf_sz = param->buf_size;

    /* Store packet processor parent ID */
    bm_pool->pp2_id = param->pp2_id;

    /* Number of buffers */
    bm_pool->bm_pool_buf_num = param->buf_num;

    /* FS_A8K Table 1558: A BPPE holds 8 x BPs (buffers), and for each
     * buffer, 2 x pointer sizes must be allocated. The BPPE region size
     * is computed by adding up all BPPEs.
     *
     * Always aligned to PP2_BPPE_UNIT_SIZE
     */
    bppe_num = bm_pool->bm_pool_buf_num;
    bppe_size = (2 * sizeof(uint64_t));
    bppe_region_size = (bppe_num * bppe_size);

    pp2_dbg("BM: pool=%u buf_num %u bppe_num %u bppe_region_size %u\n",
            bm_pool->bm_pool_id, bm_pool->bm_pool_buf_num, bppe_num, bppe_region_size);

    bm_pool->bppe_mem = cma_calloc(bppe_region_size);
    if (unlikely(!bm_pool->bppe_mem)) {
        pp2_err("BM: cannot allocate region for pool BPPEs\n");
        free(bm_pool);
        return -ENOMEM;
    }

    bm_pool->bm_pool_phys_base = (uintptr_t)cma_get_paddr(bm_pool->bppe_mem);
    bm_pool->bm_pool_virt_base = (uintptr_t)cma_get_vaddr(bm_pool->bppe_mem);

    if (!IS_ALIGNED(bm_pool->bm_pool_phys_base, MVPP2_BM_POOL_PTR_ALIGN)) {
        pp2_err("BM: pool=%u is not %u bytes aligned", param->id,
                MVPP2_BM_POOL_PTR_ALIGN);
        cma_free(bm_pool->bppe_mem);
        free(bm_pool);
        return -EIO;
    }

    pp2_dbg("BM: pool=%u BPPEs phys_base 0x%lX virt_base 0x%lX\n", bm_pool->bm_pool_id,
            bm_pool->bm_pool_phys_base, bm_pool->bm_pool_virt_base);

    pp2_inst = pp2->pp2_inst[param->pp2_id];
    cpu_slot = pp2_inst->hw.base[PP2_DEFAULT_REGSPACE].va;

    /*TODO YUVAL: Add lock here, to protect simultaneous creation of bm_pools */
    if (pp2_bm_hw_pool_create(cpu_slot, bm_pool->bm_pool_id,
                bppe_num, bm_pool->bm_pool_phys_base)) {
        pp2_err("BM: could not initialize hardware pool%u\n", bm_pool->bm_pool_id);
        cma_free(bm_pool->bppe_mem);
        free(bm_pool);
        return -EIO;
    }

    pp2_bm_pool_bufsize_set(cpu_slot, bm_pool->bm_pool_id, bm_pool->bm_pool_buf_sz);

#if PP2_BM_BUF_DEBUG
    pp2_bm_pool_print_regs(cpu_slot, bm_pool->bm_pool_id);
#endif

    return 0;
}


uint32_t pp2_bm_pool_flush(uintptr_t cpu_slot, uint32_t pool_id)
{
    uint32_t j;
    uint32_t resid_bufs = 0;
    uint32_t pool_bufs;

    resid_bufs += (pp2_reg_read(cpu_slot, MVPP2_BM_POOL_PTRS_NUM_REG(pool_id))
            & MVPP22_BM_POOL_PTRS_NUM_MASK);
    resid_bufs += (pp2_reg_read(cpu_slot, MVPP2_BM_BPPI_PTRS_NUM_REG(pool_id))
            & MVPP2_BM_BPPI_PTR_NUM_MASK);
    if (0 == resid_bufs)
        return 0;

    /* Actual number of registered buffers */
    pool_bufs = pp2_reg_read(cpu_slot, MVPP2_BM_POOL_SIZE_REG(pool_id));
    if (pool_bufs && (resid_bufs + 1) > pool_bufs) {
        /* Truncate to actual number in order to avoid
         * garbage entries from the past. Zero pool_bufs,
         * means that it is called at PPDK pre-init
         */
        resid_bufs = pool_bufs;
    }
    for (j = 0; j < (resid_bufs + 1); j++) {
        /* Clean everything */
        if (0 == pp2_bm_hw_buf_get(cpu_slot, pool_id))
            break;
    }
    resid_bufs = 0;
    resid_bufs += (pp2_reg_read(cpu_slot, MVPP2_BM_POOL_PTRS_NUM_REG(pool_id))
            & MVPP22_BM_POOL_PTRS_NUM_MASK);
    resid_bufs += (pp2_reg_read(cpu_slot, MVPP2_BM_BPPI_PTRS_NUM_REG(pool_id))
            & MVPP2_BM_BPPI_PTR_NUM_MASK);

    return resid_bufs;
}

int pp2_bm_pool_destroy(struct pp2_bm_if *bm_if,
        struct pp2_bm_pool *bm_pool)
{
    uint32_t pool_id;
    uint32_t resid_bufs = 0;

    pool_id = bm_pool->bm_pool_id;

    pp2_dbg("BM: destroying pool ID=%u\n", pool_id);

    /* If client did not clean up explicitly before
     * destroying this pool, then implictly clear up the
     * BM stack of virtual addresses by allocating
     * every available buffer from this pool
     */
    resid_bufs = pp2_bm_pool_flush(bm_if->cpu_slot, pool_id);
    if (resid_bufs) {
        pp2_dbg("BM: could not clear all buffers from pool ID=%u\n", pool_id);
        pp2_dbg("BM: total bufs    : %u\n", bm_pool->bm_pool_buf_num);
        pp2_dbg("BM: residual bufs : %u\n", resid_bufs);
    }

    pp2_bm_hw_pool_destroy(bm_if->cpu_slot, pool_id);

    cma_free(bm_pool->bppe_mem);

    free(bm_pool);

    return 0;
}

uintptr_t pp2_bm_buf_get(struct pp2_bm_if *bm_if, struct pp2_bm_pool *pool)
{
    return pp2_bm_hw_buf_get(bm_if->cpu_slot, pool->bm_pool_id);
}


void pp2_bm_pool_assign(struct pp2_port *port, uint32_t pool_id,
        uint32_t in_qid, uint32_t type)
{
    uint32_t val;
    uint32_t mask = 0;
    uint32_t offset = 0;
    uint32_t rxq_id = port->rxqs[in_qid]->id;

    if (type == BM_TYPE_LONG_BUF_POOL) {
        mask = MVPP22_RXQ_POOL_LONG_MASK;
        offset = MVPP22_RXQ_POOL_LONG_OFFS;
    } else if (type == BM_TYPE_SHORT_BUF_POOL) {
        mask = MVPP22_RXQ_POOL_SHORT_MASK;
        offset = MVPP22_RXQ_POOL_SHORT_OFFS;
    }

    val = pp2_reg_read(port->cpu_slot, MVPP2_RXQ_CONFIG_REG(rxq_id));
    val &= ~mask;
    val |= ((pool_id << offset) & mask);
    pp2_reg_write(port->cpu_slot, MVPP2_RXQ_CONFIG_REG(rxq_id), val);
}

uint32_t pp2_bm_pool_get_id(struct pp2_bm_pool *pool)
{
    return pool->bm_pool_id;
}
