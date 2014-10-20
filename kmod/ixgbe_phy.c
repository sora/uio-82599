#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/mii.h>
#include <linux/vmalloc.h>
#include <asm/io.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

#include "ixgbe_type.h"
#include "ixgbe_common.h"
#include "ixgbe_phy.h"

s32 ixgbe_reset_phy_generic(struct ixgbe_hw *hw){
        u32 i;
        u16 ctrl = 0;
        s32 status = 0;

        if (hw->phy.type == ixgbe_phy_unknown)
                status = ixgbe_identify_phy_generic(hw);

        if (status != 0 || hw->phy.type == ixgbe_phy_none)
                goto out;

        /*
         * Perform soft PHY reset to the PHY_XS.
         * This will cause a soft reset to the PHY
         */
        hw->phy.ops.write_reg(hw, IXGBE_MDIO_PHY_XS_CONTROL,
                              IXGBE_MDIO_PHY_XS_DEV_TYPE,
                              IXGBE_MDIO_PHY_XS_RESET);

        /*
         * Poll for reset bit to self-clear indicating reset is complete.
         * Some PHYs could take up to 3 seconds to complete and need about
         * 1.7 usec delay after the reset is complete.
         */
        for (i = 0; i < 30; i++) {
                msleep(100);
                hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_XS_CONTROL,
                                     IXGBE_MDIO_PHY_XS_DEV_TYPE, &ctrl);
                if (!(ctrl & IXGBE_MDIO_PHY_XS_RESET)) {
                        udelay(2);
                        break;
                }
        }

        if (ctrl & IXGBE_MDIO_PHY_XS_RESET) {
                status = IXGBE_ERR_RESET_FAILED;
        }

out:
        return status;
}

s32 ixgbe_identify_phy_generic(struct ixgbe_hw *hw)
{
        s32 status = IXGBE_ERR_PHY_ADDR_INVALID;
        u32 phy_addr;
        u16 ext_ability = 0;

printk(KERN_INFO "hw->phy.type = %d\n", hw->phy.type);
        if (hw->phy.type == ixgbe_phy_unknown) {
printk(KERN_INFO "ixgbe_phy_unknown\n");
                for (phy_addr = 0; phy_addr < IXGBE_MAX_PHY_ADDR; phy_addr++) {
                        if (ixgbe_validate_phy_addr(hw, phy_addr)) {
                                hw->phy.addr = phy_addr;
                                ixgbe_get_phy_id(hw);

				hw->phy.ops.read_reg(hw,
					IXGBE_MDIO_PHY_EXT_ABILITY,
					IXGBE_MDIO_PMA_PMD_DEV_TYPE,
					&ext_ability);
				if (ext_ability &
					(IXGBE_MDIO_PHY_10GBASET_ABILITY |
					IXGBE_MDIO_PHY_1000BASET_ABILITY))
					hw->phy.type = ixgbe_phy_cu_unknown;
				else
					hw->phy.type = ixgbe_phy_generic;

                                status = 0;
                                break;
                        }
                }

                /* Certain media types do not have a phy so an address will not
                 * be found and the code will take this path.  Caller has to
                 * decide if it is an error or not.
                 */
                if (status != 0) {
                        hw->phy.addr = 0;
                }
        } else {
                status = 0;
        }

        return status;
}

s32 ixgbe_identify_module_generic(struct ixgbe_hw *hw)
{
        s32 status = IXGBE_ERR_SFP_NOT_PRESENT;

        switch (hw->mac.ops.get_media_type(hw)) {
        case ixgbe_media_type_fiber:
		/* LAN ID is needed for sfp_type determination */
		hw->mac.ops.set_lan_id(hw);
		/* Currently we support only 10GbE SR/LR SFP+ module on the phy layer */
		if (hw->bus.lan_id == 0)
			hw->phy.sfp_type = ixgbe_sfp_type_srlr_core0;
		else
			hw->phy.sfp_type = ixgbe_sfp_type_srlr_core1;
                status = 0;
                break;

        default:
                hw->phy.sfp_type = ixgbe_sfp_type_not_present;
                status = IXGBE_ERR_SFP_NOT_PRESENT;
                break;
        }

        return status;
}

s32 ixgbe_get_sfp_init_sequence_offsets(struct ixgbe_hw *hw,
                                        u16 *list_offset,
                                        u16 *data_offset)
{
        u16 sfp_id;
        u16 sfp_type = hw->phy.sfp_type;

        if (hw->phy.sfp_type == ixgbe_sfp_type_unknown)
                return IXGBE_ERR_SFP_NOT_SUPPORTED;

        if (hw->phy.sfp_type == ixgbe_sfp_type_not_present)
                return IXGBE_ERR_SFP_NOT_PRESENT;

        /* Read offset to PHY init contents */
        if (hw->eeprom.ops.read(hw, IXGBE_PHY_INIT_OFFSET_NL, list_offset)) {
                return IXGBE_ERR_SFP_NO_INIT_SEQ_PRESENT;
        }

        if ((!*list_offset) || (*list_offset == 0xFFFF))
                return IXGBE_ERR_SFP_NO_INIT_SEQ_PRESENT;

        /* Shift offset to first ID word */
        (*list_offset)++;

        /*
         * Find the matching SFP ID in the EEPROM
         * and program the init sequence
         */
        if (hw->eeprom.ops.read(hw, *list_offset, &sfp_id))
                goto err_phy;

        while (sfp_id != IXGBE_PHY_INIT_END_NL) {
                if (sfp_id == sfp_type) {
                        (*list_offset)++;
                        if (hw->eeprom.ops.read(hw, *list_offset, data_offset))
                                goto err_phy;
                        if ((!*data_offset) || (*data_offset == 0xFFFF)) {
                                return IXGBE_ERR_SFP_NOT_SUPPORTED;
                        } else {
                                break;
                        }
                } else {
                        (*list_offset) += 2;
                        if (hw->eeprom.ops.read(hw, *list_offset, &sfp_id))
                                goto err_phy;
                }
        }

        if (sfp_id == IXGBE_PHY_INIT_END_NL) {
                return IXGBE_ERR_SFP_NOT_SUPPORTED;
        }

        return 0;

err_phy:
        return IXGBE_ERR_PHY;
}

bool ixgbe_validate_phy_addr(struct ixgbe_hw *hw, u32 phy_addr)
{
        u16 phy_id = 0;
        bool valid = false;

        hw->phy.addr = phy_addr;
        hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_ID_HIGH,
                             IXGBE_MDIO_PMA_PMD_DEV_TYPE, &phy_id);

        if (phy_id != 0xFFFF && phy_id != 0x0)
                valid = true;

printk(KERN_INFO "ixgbe_validate_phy_addr = %d\n", valid);
        return valid;
}

s32 ixgbe_get_phy_id(struct ixgbe_hw *hw)
{
        u32 status;
        u16 phy_id_high = 0;
        u16 phy_id_low = 0;

        status = hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_ID_HIGH,
                                      IXGBE_MDIO_PMA_PMD_DEV_TYPE,
                                      &phy_id_high);

        if (status == 0) {
                hw->phy.id = (u32)(phy_id_high << 16);
                status = hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_ID_LOW,
                                              IXGBE_MDIO_PMA_PMD_DEV_TYPE,
                                              &phy_id_low);
                hw->phy.id |= (u32)(phy_id_low & IXGBE_PHY_REVISION_MASK);
                hw->phy.revision = (u32)(phy_id_low & ~IXGBE_PHY_REVISION_MASK);
        }
        return status;
}

s32 ixgbe_read_phy_reg_generic(struct ixgbe_hw *hw, u32 reg_addr,
                               u32 device_type, u16 *phy_data)
{
        s32 status;
        u16 gssr;

        if (IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_LAN_ID_1)
                gssr = IXGBE_GSSR_PHY1_SM;
        else
                gssr = IXGBE_GSSR_PHY0_SM;

        if (hw->mac.ops.acquire_swfw_sync(hw, gssr) == 0) {
                status = ixgbe_read_phy_reg_mdi(hw, reg_addr, device_type,
                                                phy_data);
                hw->mac.ops.release_swfw_sync(hw, gssr);
        } else {
                status = IXGBE_ERR_SWFW_SYNC;
        }

        return status;
}

s32 ixgbe_write_phy_reg_generic(struct ixgbe_hw *hw, u32 reg_addr,
                                u32 device_type, u16 phy_data)
{
        s32 status;
        u16 gssr;

        if (IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_LAN_ID_1)
                gssr = IXGBE_GSSR_PHY1_SM;
        else
                gssr = IXGBE_GSSR_PHY0_SM;

        if (hw->mac.ops.acquire_swfw_sync(hw, gssr) == 0) {
                status = ixgbe_write_phy_reg_mdi(hw, reg_addr, device_type,
                                                 phy_data);
                hw->mac.ops.release_swfw_sync(hw, gssr);
        } else {
                status = IXGBE_ERR_SWFW_SYNC;
        }

        return status;
}

s32 ixgbe_read_phy_reg_mdi(struct ixgbe_hw *hw, u32 reg_addr, u32 device_type,
                       u16 *phy_data)
{
        u32 i, data, command;

        /* Setup and write the address cycle command */
        command = ((reg_addr << IXGBE_MSCA_NP_ADDR_SHIFT)  |
                   (device_type << IXGBE_MSCA_DEV_TYPE_SHIFT) |
                   (hw->phy.addr << IXGBE_MSCA_PHY_ADDR_SHIFT) |
                   (IXGBE_MSCA_ADDR_CYCLE | IXGBE_MSCA_MDI_COMMAND));

        IXGBE_WRITE_REG(hw, IXGBE_MSCA, command);

        /*
         * Check every 10 usec to see if the address cycle completed.
         * The MDI Command bit will clear when the operation is
         * complete
         */
        for (i = 0; i < IXGBE_MDIO_COMMAND_TIMEOUT; i++) {
                udelay(10);

                command = IXGBE_READ_REG(hw, IXGBE_MSCA);
                if ((command & IXGBE_MSCA_MDI_COMMAND) == 0)
                                break;
        }


        if ((command & IXGBE_MSCA_MDI_COMMAND) != 0) {
                return IXGBE_ERR_PHY;
        }

        /*
         * Address cycle complete, setup and write the read
         * command
         */
        command = ((reg_addr << IXGBE_MSCA_NP_ADDR_SHIFT)  |
                   (device_type << IXGBE_MSCA_DEV_TYPE_SHIFT) |
                   (hw->phy.addr << IXGBE_MSCA_PHY_ADDR_SHIFT) |
                   (IXGBE_MSCA_READ | IXGBE_MSCA_MDI_COMMAND));

        IXGBE_WRITE_REG(hw, IXGBE_MSCA, command);

        /*
         * Check every 10 usec to see if the address cycle
         * completed. The MDI Command bit will clear when the
         * operation is complete
         */
        for (i = 0; i < IXGBE_MDIO_COMMAND_TIMEOUT; i++) {
                udelay(10);

                command = IXGBE_READ_REG(hw, IXGBE_MSCA);
                if ((command & IXGBE_MSCA_MDI_COMMAND) == 0)
                        break;
        }

        if ((command & IXGBE_MSCA_MDI_COMMAND) != 0) {
                return IXGBE_ERR_PHY;
        }

        /*
         * Read operation is complete.  Get the data
         * from MSRWD
         */
        data = IXGBE_READ_REG(hw, IXGBE_MSRWD);
        data >>= IXGBE_MSRWD_READ_DATA_SHIFT;
        *phy_data = (u16)(data);

        return 0;
}

s32 ixgbe_write_phy_reg_mdi(struct ixgbe_hw *hw, u32 reg_addr,
                                u32 device_type, u16 phy_data)
{
        u32 i, command;

        /* Put the data in the MDI single read and write data register*/
        IXGBE_WRITE_REG(hw, IXGBE_MSRWD, (u32)phy_data);

        /* Setup and write the address cycle command */
        command = ((reg_addr << IXGBE_MSCA_NP_ADDR_SHIFT)  |
                   (device_type << IXGBE_MSCA_DEV_TYPE_SHIFT) |
                   (hw->phy.addr << IXGBE_MSCA_PHY_ADDR_SHIFT) |
                   (IXGBE_MSCA_ADDR_CYCLE | IXGBE_MSCA_MDI_COMMAND));

        IXGBE_WRITE_REG(hw, IXGBE_MSCA, command);

        /*
         * Check every 10 usec to see if the address cycle completed.
         * The MDI Command bit will clear when the operation is
         * complete
         */
        for (i = 0; i < IXGBE_MDIO_COMMAND_TIMEOUT; i++) {
                udelay(10);

                command = IXGBE_READ_REG(hw, IXGBE_MSCA);
                if ((command & IXGBE_MSCA_MDI_COMMAND) == 0)
                        break;
        }

        if ((command & IXGBE_MSCA_MDI_COMMAND) != 0) {
                return IXGBE_ERR_PHY;
        }

        /*
         * Address cycle complete, setup and write the write
         * command
         */
        command = ((reg_addr << IXGBE_MSCA_NP_ADDR_SHIFT)  |
                   (device_type << IXGBE_MSCA_DEV_TYPE_SHIFT) |
                   (hw->phy.addr << IXGBE_MSCA_PHY_ADDR_SHIFT) |
                   (IXGBE_MSCA_WRITE | IXGBE_MSCA_MDI_COMMAND));

        IXGBE_WRITE_REG(hw, IXGBE_MSCA, command);

        /*
         * Check every 10 usec to see if the address cycle
         * completed. The MDI Command bit will clear when the
         * operation is complete
         */
        for (i = 0; i < IXGBE_MDIO_COMMAND_TIMEOUT; i++) {
                udelay(10);

                command = IXGBE_READ_REG(hw, IXGBE_MSCA);
                if ((command & IXGBE_MSCA_MDI_COMMAND) == 0)
                        break;
        }

        if ((command & IXGBE_MSCA_MDI_COMMAND) != 0) {
                return IXGBE_ERR_PHY;
        }

        return 0;
}

