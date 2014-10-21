#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/sysctl.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <asm/io.h>

#include "ixgbe_type.h"
#include "ixgbe_common.h"
#include "ixgbe_82599.h"
#include "uio_ixgbe.h"

char uio_ixgbe_driver_name[]    = "uio-ixgbe for 82599";
char uio_ixgbe_driver_string[]  = "UIO-IXGBE kernel backend";
char uio_ixgbe_driver_version[] = "1.0";
char uio_ixgbe_copyright[]      = "Copyright (c) 2009 Qualcomm Inc.";

static DEFINE_PCI_DEVICE_TABLE(uio_ixgbe_pci_tbl) = {
        {PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP)},
        /* required last entry */
        {0, }
};

MODULE_DEVICE_TABLE(pci, uio_ixgbe_pci_tbl);

static LIST_HEAD(dev_list);
static DEFINE_SEMAPHORE(dev_sem);

static int uio_ixgbe_down(struct uio_ixgbe_udapter *ud);
static void uio_ixgbe_populate_info(struct uio_ixgbe_udapter *ud, struct uio_ixgbe_info *info);
void uio_ixgbe_reset(struct uio_ixgbe_udapter *ud);

u16 ixgbe_read_pci_cfg_word(struct ixgbe_hw *hw, u32 reg){
        struct uio_ixgbe_udapter *ud = hw->back;
        u16 value;

        if (unlikely(!hw->hw_addr))
                return IXGBE_FAILED_READ_CFG_WORD;

        pci_read_config_word(ud->pdev, reg, &value);

        return value;
}

static int uio_ixgbe_alloc_enid(void){
        struct uio_ixgbe_udapter *ud;
        unsigned int id = 0;

        list_for_each_entry(ud, &dev_list, list) {
                if (ud->id != id)
                        return id;
                id++;
        }

        return id;
}

static struct uio_ixgbe_udapter *uio_ixgbe_lookup_enid(char *_id)
{
        struct uio_ixgbe_udapter *ud;
        unsigned int id = simple_strtol(_id, NULL, 10);

        list_for_each_entry(ud, &dev_list, list) {
                if (ud->id == id)
                        return ud;
        }

        return NULL;
}

static struct uio_ixgbe_udapter *uio_ixgbe_lookup_pciid(char *id)
{
        struct uio_ixgbe_udapter *ud;

        list_for_each_entry(ud, &dev_list, list) {
                if (!strcmp(pci_name(ud->pdev), id))
                        return ud;
        }

        return NULL;
}

static struct uio_ixgbe_udapter *uio_ixgbe_lookup_ud(char *id)
{
        switch (id[0]) {
        case '#':
                return uio_ixgbe_lookup_enid(id + 1);

        default:
                return uio_ixgbe_lookup_pciid(id);
        }
}

static int uio_ixgbe_udapter_inuse(struct uio_ixgbe_udapter *ud)
{
	unsigned ref = atomic_read(&ud->refcount);
	if (ref == 1)
		return 0;
	return 1;
}

static void uio_ixgbe_udapter_get(struct uio_ixgbe_udapter *ud)
{
	atomic_inc(&ud->refcount);
}

static void uio_ixgbe_udapter_put(struct uio_ixgbe_udapter *ud)
{
	if (atomic_dec_and_test(&ud->refcount))
		kfree(ud);
}

static struct uio_ixgbe_udapter *uio_ixgbe_udapter_alloc(void)
{
	struct uio_ixgbe_udapter *ud;

	ud = kmalloc(sizeof(struct uio_ixgbe_udapter), GFP_KERNEL);
	if (!ud){
		return NULL;
	}
	memset(ud, 0, sizeof(struct uio_ixgbe_udapter));

	ud->hw = kmalloc(sizeof(struct ixgbe_hw), GFP_KERNEL);
	if(!ud->hw){
		return NULL;
	}
	memset(ud->hw, 0, sizeof(struct ixgbe_hw));

	atomic_set(&ud->refcount, 1);

	spin_lock_init(&ud->lock);
	sema_init(&ud->sem,1);
	init_waitqueue_head(&ud->read_wait);
	return ud;
}

static void uio_ixgbe_release_hw_control(struct uio_ixgbe_udapter *ud){
	struct ixgbe_hw *hw = ud->hw;
        u32 ctrl_ext;

        /* Let firmware take over control of h/w */
        ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
        IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT,
                        ctrl_ext & ~IXGBE_CTRL_EXT_DRV_LOAD);
}

static void uio_ixgbe_take_hw_control(struct uio_ixgbe_udapter *ud){
	struct ixgbe_hw *hw = ud->hw;
        u32 ctrl_ext;

        /* Let firmware know the driver has taken over */
        ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
        IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT,
                        ctrl_ext | IXGBE_CTRL_EXT_DRV_LOAD);
}

static int uio_ixgbe_sw_init(struct uio_ixgbe_udapter *ud){
	struct ixgbe_hw *hw = ud->hw;
	struct pci_dev *pdev = ud->pdev;

        /* PCI config space info */
        hw->vendor_id = pdev->vendor;
        hw->device_id = pdev->device;
        hw->revision_id = pdev->revision;
        hw->subsystem_vendor_id = pdev->subsystem_vendor;
        hw->subsystem_device_id = pdev->subsystem_device;

        /* Setup hw api */
	hw->mac.type  = ixgbe_mac_82599EB;
        ixgbe_init_ops_82599(hw);

	return 0;
}

static int uio_ixgbe_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct uio_ixgbe_udapter *ud;
	struct ixgbe_hw *hw;
        u16 offset = 0, eeprom_verh = 0, eeprom_verl = 0;
        u16 eeprom_cfg_blkh = 0, eeprom_cfg_blkl = 0;
        u32 etrack_id;
        u16 build, major, patch;
	u8 part_str[IXGBE_PBANUM_LENGTH];
	int pci_using_dac, err;

	IXGBE_DBG("probing device %s\n", pci_name(pdev));

        err = pci_enable_device_mem(pdev);
        if (err)
                return err;

        if (!dma_set_mask((struct device *)pdev, DMA_BIT_MASK(64)) &&
            !dma_set_coherent_mask((struct device *)pdev, DMA_BIT_MASK(64))) {
                pci_using_dac = 1;
        } else {
                err = dma_set_mask((struct device *)pdev, DMA_BIT_MASK(32));
                if (err) {
                        err = dma_set_coherent_mask((struct device *)pdev,
                                                    DMA_BIT_MASK(32));
                        if (err) {
                                dev_err((struct device *)pdev, "No usable DMA "
                                        "configuration, aborting\n");
                                goto err_dma;
                        }
                }
                pci_using_dac = 0;
        }

        err = pci_request_selected_regions(pdev, pci_select_bars(pdev,
                                           IORESOURCE_MEM), uio_ixgbe_driver_name);
        if (err) {
                IXGBE_ERR("pci_request_selected_regions failed 0x%x\n", err);
                goto err_pci_reg;
        }

	ud = uio_ixgbe_udapter_alloc();
	if (!ud) {
		err = -ENOMEM;
		goto err_alloc;
	}
	hw = ud->hw;

	pci_set_drvdata(pdev, ud);
	ud->pdev = pdev;
	hw->back = ud;

	pci_set_master(pdev);
	pci_save_state(pdev);

	ud->iobase = pci_resource_start(pdev, 0);
	ud->iolen  = pci_resource_len(pdev,   0);

	hw->hw_addr = ioremap(ud->iobase, ud->iolen);
	if (!hw->hw_addr) {
		err = -EIO;
		goto err_ioremap;
	}

	/* SOFTWARE INITIALIZATION */
        err = uio_ixgbe_sw_init(ud);
        if (err)
                goto err_sw_init;

        /* reset_hw fills in the perm_addr as well */
        err = hw->mac.ops.reset_hw(hw);

        if (err) {
                IXGBE_ERR("HW Init failed: %d\n", err);
                goto err_sw_init;
        }

        /* make sure the EEPROM is good */
        if (hw->eeprom.ops.validate_checksum &&
            (hw->eeprom.ops.validate_checksum(hw, NULL) < 0)) {
                IXGBE_ERR("The EEPROM Checksum Is Not Valid\n");
                err = -EIO;
                goto err_sw_init;
        }

        if (ixgbe_validate_mac_addr(hw->mac.perm_addr)) {
                IXGBE_ERR("invalid MAC address\n");
                err = -EIO;
                goto err_sw_init;
        }

       /*
         * Save off EEPROM version number and Option Rom version which
         * together make a unique identify for the eeprom
         */
        hw->eeprom.ops.read(hw, 0x2e, &eeprom_verh);
        hw->eeprom.ops.read(hw, 0x2d, &eeprom_verl);
        etrack_id = (eeprom_verh << 16) | eeprom_verl;

        hw->eeprom.ops.read(hw, 0x17, &offset);

        /* Make sure offset to SCSI block is valid */
        if (!(offset == 0x0) && !(offset == 0xffff)) {
                hw->eeprom.ops.read(hw, offset + 0x84, &eeprom_cfg_blkh);
                hw->eeprom.ops.read(hw, offset + 0x83, &eeprom_cfg_blkl);

                /* Only display Option Rom if exist */
                if (eeprom_cfg_blkl && eeprom_cfg_blkh) {
                        major = eeprom_cfg_blkl >> 8;
                        build = (eeprom_cfg_blkl << 8) | (eeprom_cfg_blkh >> 8);
                        patch = eeprom_cfg_blkh & 0x00ff;

                        snprintf(ud->eeprom_id, sizeof(ud->eeprom_id),
                                 "0x%08x, %d.%d.%d", etrack_id, major, build,
                                 patch);
                } else {
                        snprintf(ud->eeprom_id, sizeof(ud->eeprom_id),
                                 "0x%08x", etrack_id);
                }
        } else {
                snprintf(ud->eeprom_id, sizeof(ud->eeprom_id),
                         "0x%08x", etrack_id);
        }

        /* reset the hardware with the new settings */
        err = hw->mac.ops.start_hw(hw);
        if (err == IXGBE_ERR_EEPROM_VERSION) {
                /* We are running on a pre-production device, log a warning */
                IXGBE_INFO("This device is a pre-production adapter/LOM. "
                           "Please be aware there may be issues associated "
                           "with your hardware.  If you are experiencing "
                           "problems please contact your Intel or hardware "
                           "representative who provided you with this "
                           "hardware.\n");
        }

        /* power down the optics for 82599 SFP+ fiber */
        if (hw->mac.ops.disable_tx_laser)
                hw->mac.ops.disable_tx_laser(hw);

        /* First try to read PBA as a string */
        err = ixgbe_read_pba_string_generic(hw, part_str, IXGBE_PBANUM_LENGTH);
        if (err)
                strncpy(part_str, "Unknown", IXGBE_PBANUM_LENGTH);
        if (hw->phy.sfp_type != ixgbe_sfp_type_not_present)
                IXGBE_INFO("MAC: %d, PHY: %d, SFP+: %d, PBA No: %s\n",
                       hw->mac.type, hw->phy.type, hw->phy.sfp_type, part_str);

        IXGBE_INFO("%02x:%02x:%02x:%02x:%02x:%02x\n",
                   hw->mac.perm_addr[0], hw->mac.perm_addr[1],
                   hw->mac.perm_addr[2], hw->mac.perm_addr[3],
                   hw->mac.perm_addr[4], hw->mac.perm_addr[5]);

        /* firmware requires blank driver version */
        if (hw->mac.ops.set_fw_drv_ver)
                hw->mac.ops.set_fw_drv_ver(hw, 0xFF, 0xFF, 0xFF, 0xFF);

	/* Add to global list */
	down(&dev_sem);
	ud->id = uio_ixgbe_alloc_enid();
	list_add(&ud->list, &dev_list);
	up(&dev_sem);

	IXGBE_INFO("device[%u] %s initialized\n", ud->id, pci_name(pdev));
	IXGBE_INFO("device[%u] %s dma mask %llx\n", ud->id, pci_name(pdev),
		(unsigned long long) pdev->dma_mask);

	return 0;

err_sw_init:
	iounmap(hw->hw_addr);
err_ioremap:
	kfree(ud->hw);
	kfree(ud);
err_alloc:
        pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
err_pci_reg:
err_dma:

	return err;
}

static void uio_ixgbe_remove(struct pci_dev *pdev){
	struct uio_ixgbe_udapter *ud = pci_get_drvdata(pdev);
	struct ixgbe_hw *hw = ud->hw;

	if(ud->up){
		uio_ixgbe_down(ud);
	}

	iounmap(hw->hw_addr);
	pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	pci_disable_device(pdev);

	down(&dev_sem);
	list_del(&ud->list);
	ud->removed = 1;
	up(&dev_sem);

	IXGBE_INFO("device[%u] %s removed\n", ud->id, pci_name(pdev));

	wake_up_interruptible(&ud->read_wait);
	uio_ixgbe_udapter_put(ud);
}

static pci_ers_result_t uio_ixgbe_io_error_detected(struct pci_dev *pdev, pci_channel_state_t state)
{
	// FIXME: Do something
	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t uio_ixgbe_io_slot_reset(struct pci_dev *pdev)
{
	// FIXME: Do something
	return PCI_ERS_RESULT_RECOVERED;
}

static void uio_ixgbe_io_resume(struct pci_dev *pdev)
{
        // FIXME: Do something
        return;
}

/*
 * Interrupt handler.
 * MSI-X mode is not supported.
 */
static irqreturn_t uio_ixgbe_interrupt(int irq, void *data)
{
	struct uio_ixgbe_udapter *ud = data;
	struct ixgbe_hw *hw = ud->hw;
	uint32_t eicr;

	eicr = IXGBE_READ_REG(hw, IXGBE_EICR);
	if (unlikely(!eicr))
		return IRQ_NONE;  /* Not our interrupt */

	(void) xchg(&ud->eicr, eicr);

	/* 
	 * We setup EIAM such that interrupts are auto-masked (disabled).
	 * User-space will re-enable them.
	 */

	wake_up_interruptible(&ud->read_wait);
	return IRQ_HANDLED;
}

void uio_ixgbe_write_eitr(struct uio_ixgbe_udapter *ud, int vector){
        struct ixgbe_hw *hw = ud->hw;
        u32 itr_reg = IXGBE_20K_ITR & IXGBE_MAX_EITR;

        /*
         * set the WDIS bit to not clear the timer bits and cause an
         * immediate assertion of the interrupt
         */
        itr_reg |= IXGBE_EITR_CNT_WDIS;
        IXGBE_WRITE_REG(hw, IXGBE_EITR(vector), itr_reg);
}

static void uio_ixgbe_set_ivar(struct uio_ixgbe_udapter *ud, s8 direction, u8 queue, u8 msix_vector){
        u32 ivar, index;
        struct ixgbe_hw *hw = ud->hw;

        /* tx or rx causes */
        msix_vector |= IXGBE_IVAR_ALLOC_VAL;
        index = ((16 * (queue & 1)) + (8 * direction));
        ivar = IXGBE_READ_REG(hw, IXGBE_IVAR(queue >> 1));
        ivar &= ~(0xFF << index);
        ivar |= (msix_vector << index);
        IXGBE_WRITE_REG(hw, IXGBE_IVAR(queue >> 1), ivar);
}

static void uio_ixgbe_configure_msix(struct uio_ixgbe_udapter *ud){
	int vector, v_budget, err;
	struct ixgbe_hw *hw = ud->hw;

	v_budget = min_t(int, MIN_MSIX_Q_VECTORS, hw->mac.max_msix_vectors);
	ud->msix_entries = kcalloc(v_budget, sizeof(struct msix_entry), GFP_KERNEL);
	err = v_budget;

        if (!ud->msix_entries) {
		/* should return immediately */
	}

        for (vector = 0; vector < v_budget; vector++){
                ud->msix_entries[vector].entry = vector;
	}

	while (err){
		/* err == number of vectors we should try again with */ 
              	err = pci_enable_msix(ud->pdev, ud->msix_entries, err);

		if(err < 0){
                       	/* Nasty failure */
			v_budget = 0;
			break;
               	}
        }

	ud->num_q_vectors = v_budget;

	for (vector = 0; vector < ud->num_q_vectors; vector++){
		struct msix_entry *entry = &ud->msix_entries[vector];
                err = request_irq(entry->vector, &uio_ixgbe_interrupt, 0, pci_name(ud->pdev), ud);
		if(err){
			/* should return immediately */
		}

		uio_ixgbe_set_ivar(ud, 0, 0, entry->vector);
		uio_ixgbe_set_ivar(ud, 1, 0, entry->vector);

		uio_ixgbe_write_eitr(ud, entry->vector);
	}

	return;
}

static void uio_ixgbe_setup_gpie(struct uio_ixgbe_udapter *ud){
	struct ixgbe_hw *hw = ud->hw;
        u32 gpie = 0;

        gpie = IXGBE_GPIE_MSIX_MODE | IXGBE_GPIE_PBA_SUPPORT | IXGBE_GPIE_OCD;
        gpie |= IXGBE_GPIE_EIAME;

        /*
         * use EIAM to auto-mask when MSI-X interrupt is asserted
         * this saves a register write for every interrupt
         */
        IXGBE_WRITE_REG(hw, IXGBE_EIAM_EX(0), 0xFFFFFFFF);
        IXGBE_WRITE_REG(hw, IXGBE_EIAM_EX(1), 0xFFFFFFFF);
        IXGBE_WRITE_REG(hw, IXGBE_GPIE, gpie);

	return;
}

static int uio_ixgbe_up(struct uio_ixgbe_udapter *ud){
        struct ixgbe_hw *hw = ud->hw;

        uio_ixgbe_take_hw_control(ud);
        uio_ixgbe_setup_gpie(ud);

        /* set up msix interupt */
        uio_ixgbe_configure_msix(ud);

        /* enable the optics for 82599 SFP+ fiber */
        if (hw->mac.ops.enable_tx_laser)
                hw->mac.ops.enable_tx_laser(hw);

        if (hw->mac.ops.setup_link)
                hw->mac.ops.setup_link(hw, IXGBE_LINK_SPEED_10GB_FULL, true);

        /* clear any pending interrupts, may auto mask */
        IXGBE_READ_REG(hw, IXGBE_EICR);

	/* User space application enables interrupts after */
	ud->up = 1;

        return 0;
}

static int uio_ixgbe_cmd_up(struct file *file, void __user *argp){
	struct uio_ixgbe_udapter *ud = file->private_data;
	struct uio_ixgbe_open_req req;
	int err = 0;

        if(ud->removed){
                return -ENODEV;
        }

	if(ud->up){
		return -EALREADY;
	}

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	IXGBE_DBG("open req\n");
        err = uio_ixgbe_up(ud);
        if (err){
                goto err_up_complete;
	}

	uio_ixgbe_populate_info(ud, &req.info);

	if (copy_to_user(argp, &req, sizeof(req))) {
		err = -EFAULT;
		goto err_up_complete;
	}

	return 0;

err_up_complete:
	uio_ixgbe_down(ud);

	return err;
}

static int uio_ixgbe_down(struct uio_ixgbe_udapter *ud){
	struct ixgbe_hw *hw = ud->hw;
	int vector;

	/* Disable Interrupts */
	IXGBE_WRITE_REG(hw, IXGBE_EIMC, 0xFFFF0000);
	IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(0), ~0);
	IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(1), ~0);

	IXGBE_WRITE_FLUSH(hw);

	for(vector = 0; vector < ud->num_q_vectors; vector++){
		synchronize_irq(ud->msix_entries[vector].vector);
	}

	/* Disable Transmits */
	IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(0), IXGBE_TXDCTL_SWFLSH);

	/* Disable the Tx DMA engine on 82599 and X540 */
	IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL, (IXGBE_READ_REG(hw, IXGBE_DMATXCTL) & ~IXGBE_DMATXCTL_TE));

	/* Reset hardware */
        if (!pci_channel_offline(ud->pdev)){
                uio_ixgbe_reset(ud);
	}

        /* power down the optics for 82599 SFP+ fiber */
        if (hw->mac.ops.disable_tx_laser){
                hw->mac.ops.disable_tx_laser(hw);
	}

	/* free irqs */
	for (vector = 0; vector < ud->num_q_vectors; vector++) {
		struct msix_entry *entry = &ud->msix_entries[vector];
		free_irq(entry->vector, ud);
	}

	uio_ixgbe_release_hw_control(ud);
	ud->up = 0;
	return 0;
}

static int uio_ixgbe_cmd_down(struct uio_ixgbe_udapter *ud, unsigned long arg){
        if(ud->removed){
                return -ENODEV;
        }

        if(!ud->up){
                return -EALREADY;
        }

	IXGBE_DBG("down req\n");
	uio_ixgbe_down(ud);

	return 0;
}

void uio_ixgbe_reset(struct uio_ixgbe_udapter *ud){
        struct ixgbe_hw *hw = ud->hw;
        int err;

        err = hw->mac.ops.init_hw(hw);
        switch (err) {
        case 0:
        case IXGBE_ERR_SFP_NOT_PRESENT:
        case IXGBE_ERR_SFP_NOT_SUPPORTED:
                break;
        case IXGBE_ERR_MASTER_REQUESTS_PENDING:
                IXGBE_ERR("master disable timed out\n");
                break;
        case IXGBE_ERR_EEPROM_VERSION:
                /* We are running on a pre-production device, log a warning */
                IXGBE_ERR("This device is a pre-production adapter/LOM. "
                           "Please be aware there may be issues associated "
                           "with your hardware.  If you are experiencing "
                           "problems please contact your Intel or hardware "
                           "representative who provided you with this "
                           "hardware.\n");
                break;
        default:
                IXGBE_ERR("Hardware Error: %d\n", err);
        }

        return;
}

static int uio_ixgbe_cmd_reset(struct uio_ixgbe_udapter *ud, unsigned long arg){
        if (ud->removed){
                return -ENODEV;
        }

        if(!ud->up){
                return 0;
        }

	IXGBE_DBG("reset req\n");
	uio_ixgbe_reset(ud);

	return 0;
}


static int uio_ixgbe_cmd_bind(struct file *file, void __user *argp)
{
        struct uio_ixgbe_udapter *ud;
        struct uio_ixgbe_bind_req req;
        int err;

        if (copy_from_user(&req, argp, sizeof(req)))
                return -EFAULT;

        IXGBE_DBG("bind req name %s\n", req.name);

        down(&dev_sem);

        ud = uio_ixgbe_lookup_ud(req.name);
        if (!ud) {
                err = -ENOENT;
                goto out;
        }

        // Only one process is alowed to bind
        if (uio_ixgbe_udapter_inuse(ud)) {
                err = -EBUSY;
                goto out;
        }

        uio_ixgbe_udapter_get(ud);
        file->private_data = ud;
        err = 0;

out:
        up(&dev_sem);
        return err;
}

static int uio_ixgbe_cmd_check_link(struct uio_ixgbe_udapter *ud, void __user *argp){
	struct ixgbe_hw *hw = ud->hw;
	struct uio_ixgbe_link_req req;
	int err = 0, flush = 0;
       	bool link_up;
        u32 link_speed = 0;

	if (!ud->up)
		return -EAGAIN;

	/* Here we do link management that uio_ixgbe_watchdog normally does */
        if (hw->mac.ops.check_link) {
                hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
        } else {
                /* always assume link is up, if no check link function */
                link_speed = IXGBE_LINK_SPEED_10GB_FULL;
                link_up = true;
        }

	IXGBE_DBG("check_link req speed %u up %u\n", link_speed, link_up);

	if (link_up) {
		if (!ud->link_speed) {
			ud->link_speed  = link_speed;
			ud->link_duplex = 2;

			IXGBE_DBG("link is up %d mbps %s\n", ud->link_speed,
				ud->link_duplex == 2 ?  "Full Duplex" : "Half Duplex");
		}
	} else {
		if (ud->link_speed) {
			ud->link_speed  = 0;
			ud->link_duplex = 0;

			/* We've lost the link, so the controller stops DMA,
			 * but we've got queued Tx/Rx work that's never going
			 * to get done. Tell the app to flush RX */
			flush = 1;

			IXGBE_DBG("link is down\n");
		}
	}

	req.speed   = ud->link_speed;
	req.duplex  = ud->link_duplex;
	req.flush   = flush;

	if (copy_to_user(argp, &req, sizeof(req)))
		err = -EFAULT;

	return err;
}

static int uio_ixgbe_cmd_get_link(struct uio_ixgbe_udapter *ud, void __user *argp)
{
	struct uio_ixgbe_link_req req;
	int err = 0;

	IXGBE_DBG("get_link req\n");

	req.speed       = ud->link_speed;
	req.duplex      = ud->link_duplex;
	req.flush       = 0;
	if (copy_to_user(argp, &req, sizeof(req)))
		err = -EFAULT;

	return err;
}

static int uio_ixgbe_cmd_set_link(struct uio_ixgbe_udapter *ud, void __user *argp)
{
	struct uio_ixgbe_link_req req;

	if (!ud->up)
		return -EAGAIN;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	IXGBE_DBG("set_link req\n");

	/* 
	 * Ignore this for now.
	 * Does not make too much sense to mess with link speed on 10Gbe cards.
	 */ 

	return 0;
}

static void uio_ixgbe_populate_info(struct uio_ixgbe_udapter *ud, struct uio_ixgbe_info *info){
	struct ixgbe_hw *hw = ud->hw;

        info->irq       = ud->pdev->irq;
        info->mmio_base = ud->iobase;
        info->mmio_size = ud->iolen;

        memcpy(info->mac_addr, hw->mac.perm_addr, ETH_ALEN);
        info->mac_type = hw->mac.type;
        info->phy_type = hw->phy.type;
}

static int uio_ixgbe_cmd_info(struct file *file, void __user *argp)
{
        struct uio_ixgbe_udapter *ud;
        struct uio_ixgbe_info_req req;
        int err;

        if (copy_from_user(&req, argp, sizeof(req)))
                return -EFAULT;

        IXGBE_DBG("info req name %s\n", req.name);

        down(&dev_sem);

        ud = uio_ixgbe_lookup_ud(req.name);
        if (!ud) {
                err = -ENOENT;
                goto out;
        }

        err = 0;
        uio_ixgbe_populate_info(ud, &req.info);

        if (copy_to_user(argp, &req, sizeof(req))) {
                err = -EFAULT;
                goto out;
        }

out:
        up(&dev_sem);
        return err;
}

static long uio_ixgbe_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct uio_ixgbe_udapter *ud = file->private_data;
	void __user * argp = (void __user *) arg;
	int err;

	IXGBE_DBG("ioctl cmd=%d arg=%lu ud=%p\n", cmd, arg, ud);

	if (cmd == UIO_IXGBE_INFO)
		return uio_ixgbe_cmd_info(file, argp);

	if (cmd == UIO_IXGBE_BIND) {
		if (ud)
			return -EALREADY;

		return uio_ixgbe_cmd_bind(file, argp);
	}

	if (!ud)
		return -EBADFD;

	down(&ud->sem);

	switch (cmd) {
	case UIO_IXGBE_UP:
		err = uio_ixgbe_cmd_up(file, argp);
		break;

	case UIO_IXGBE_DOWN:
		err = uio_ixgbe_cmd_down(ud, arg);
		break;

	case UIO_IXGBE_RESET:
		err = uio_ixgbe_cmd_reset(ud, arg);
		break;

	case UIO_IXGBE_CHECK_LINK:
		err = uio_ixgbe_cmd_check_link(ud, argp);
		break;

	case UIO_IXGBE_SET_LINK:
		err = uio_ixgbe_cmd_set_link(ud, argp);
		break;

	case UIO_IXGBE_GET_LINK:
		err = uio_ixgbe_cmd_get_link(ud, argp);
		break;

	default:
		err = -EINVAL;
		break;
	};

	up(&ud->sem);

	return err;
}

static unsigned int uio_ixgbe_poll(struct file *file, poll_table *wait)
{
        unsigned int mask = POLLOUT | POLLWRNORM;
        struct uio_ixgbe_udapter *ud = file->private_data;
        if (!ud)
                return -EBADFD;

        IXGBE_DBG("poll eicr=0x%x\n", ud->eicr);

        poll_wait(file, &ud->read_wait, wait);
        if (ud->eicr)
                mask |= POLLIN | POLLRDNORM;

        return mask;
}

static ssize_t uio_ixgbe_read(struct file * file, char __user * buf,
                            size_t count, loff_t *pos)
{
        struct uio_ixgbe_udapter *ud = file->private_data;
        DECLARE_WAITQUEUE(wait, current);
        uint32_t eicr;
        int err;

        if (!ud)
                return -EBADFD;

        if (count < sizeof(eicr))
                return -EINVAL;

        add_wait_queue(&ud->read_wait, &wait);
        while (count) {
                set_current_state(TASK_INTERRUPTIBLE);

                if (ud->removed) {
                        err = -ENODEV;
                        break;
                }

                eicr = xchg(&ud->eicr, 0);

                IXGBE_DBG("read eicr 0x%x\n", eicr);

                if (!eicr) {
                        if (file->f_flags & O_NONBLOCK) {
                                err = -EAGAIN;
                                break;
                        }
                        if (signal_pending(current)) {
                                err = -ERESTARTSYS;
                                break;
                        }

                        /* Nothing to read, let's sleep */
                        schedule();
                        continue;
                }
                if (copy_to_user(buf, &eicr, sizeof(eicr)))
                        err = -EFAULT;
                else
                        err = sizeof(eicr);
                break;
        }

        __set_current_state(TASK_RUNNING);
        remove_wait_queue(&ud->read_wait, &wait);

        return err;
}

static ssize_t uio_ixgbe_write(struct file * file, const char __user * buf,
                             size_t count, loff_t *pos)
{
        return 0;
}

static int uio_ixgbe_open(struct inode *inode, struct file * file)
{
	file->private_data = NULL;
	return 0;
}

static int uio_ixgbe_close(struct inode *inode, struct file *file)
{
	struct uio_ixgbe_udapter *ud = file->private_data;
	if (!ud)
		return 0;

	IXGBE_DBG("close ud=%p\n", ud);

	down(&ud->sem);
	uio_ixgbe_cmd_down(ud, 0);
	up(&ud->sem);

	uio_ixgbe_udapter_put(ud);
	return 0;
}

static void uio_ixgbe_vm_open(struct vm_area_struct *vma)
{
}

static void uio_ixgbe_vm_close(struct vm_area_struct *vma)
{
}

static int uio_ixgbe_vm_fault(struct vm_area_struct *area, struct vm_fault *fdata)
{
	return VM_FAULT_SIGBUS;
}

static struct vm_operations_struct uio_ixgbe_mmap_ops = {
	.open   = uio_ixgbe_vm_open,
	.close  = uio_ixgbe_vm_close,
	.fault  = uio_ixgbe_vm_fault
};

static int uio_ixgbe_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct uio_ixgbe_udapter *ud = file->private_data;

	unsigned long start = vma->vm_start;
	unsigned long size  = vma->vm_end - vma->vm_start;
	unsigned long pgoff = vma->vm_pgoff;
	unsigned long pfn;

	if (!ud)
		return -ENODEV;

	IXGBE_DBG("mmap ud %p start %lu size %lu pgoff %lu\n", ud, start, size, pgoff);

	if (pgoff)
		return -EINVAL;

	pfn = ud->iobase >> PAGE_SHIFT;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, start, pfn, size, vma->vm_page_prot))
		return -EAGAIN;

	vma->vm_ops = &uio_ixgbe_mmap_ops;
	return 0;
}

static struct file_operations uio_ixgbe_fops = {
	.owner	 = THIS_MODULE,
	.llseek  = no_llseek,
	.read	 = uio_ixgbe_read,
	.write	 = uio_ixgbe_write,
	.poll	 = uio_ixgbe_poll,
	.open	 = uio_ixgbe_open,
	.release = uio_ixgbe_close,
	.mmap	 = uio_ixgbe_mmap,
	.unlocked_ioctl = uio_ixgbe_ioctl,
};

static struct miscdevice uio_ixgbe_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "uio-ixgbe",
	.fops = &uio_ixgbe_fops,
};

static struct pci_error_handlers uio_ixgbe_err_handler = {
	.error_detected = uio_ixgbe_io_error_detected,
	.slot_reset     = uio_ixgbe_io_slot_reset,
	.resume         = uio_ixgbe_io_resume,
};

static struct pci_driver uio_ixgbe_driver = {
	.name        = uio_ixgbe_driver_name,
	.id_table    = uio_ixgbe_pci_tbl,
	.probe       = uio_ixgbe_probe,
	.remove      = uio_ixgbe_remove,
	.err_handler = &uio_ixgbe_err_handler,
};

static int __init uio_ixgbe_init_module(void)
{
	int err;

	printk(KERN_INFO "%s - version %s\n", uio_ixgbe_driver_string, uio_ixgbe_driver_version);
	printk(KERN_INFO "%s\n", uio_ixgbe_copyright);

	err = pci_register_driver(&uio_ixgbe_driver);
	if (!err) {
		err = misc_register(&uio_ixgbe_miscdev);
		if (err) {
			IXGBE_ERR("failed to register misc device\n");
			pci_unregister_driver(&uio_ixgbe_driver);
			return err;
		}
	}

	return err;
}

static void __exit uio_ixgbe_exit_module(void)
{
	misc_deregister(&uio_ixgbe_miscdev);
	pci_unregister_driver(&uio_ixgbe_driver);
}

module_init(uio_ixgbe_init_module);
module_exit(uio_ixgbe_exit_module);

/* ---- */

MODULE_AUTHOR("Maxim Krasnyanskiy <maxk@qualcomm.com>");
MODULE_DESCRIPTION("UIO-IXGBE kernel backend");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
