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

static void ixgbe_i2c_start(struct ixgbe_hw *hw);
static void ixgbe_i2c_stop(struct ixgbe_hw *hw);
static s32 ixgbe_clock_in_i2c_byte(struct ixgbe_hw *hw, u8 *data);
static s32 ixgbe_clock_out_i2c_byte(struct ixgbe_hw *hw, u8 data);
static s32 ixgbe_get_i2c_ack(struct ixgbe_hw *hw);
static void ixgbe_raise_i2c_clk(struct ixgbe_hw *hw, u32 *i2cctl);
static void ixgbe_lower_i2c_clk(struct ixgbe_hw *hw, u32 *i2cctl);
static s32 ixgbe_clock_in_i2c_bit(struct ixgbe_hw *hw, bool *data);
static s32 ixgbe_clock_out_i2c_bit(struct ixgbe_hw *hw, bool data);
static bool ixgbe_get_i2c_data(u32 *i2cctl);
static s32 ixgbe_set_i2c_data(struct ixgbe_hw *hw, u32 *i2cctl, bool data);
static void ixgbe_i2c_bus_clear(struct ixgbe_hw *hw);

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

        if (hw->phy.type == ixgbe_phy_unknown) {
                for (phy_addr = 0; phy_addr < IXGBE_MAX_PHY_ADDR; phy_addr++) {
                        if (ixgbe_validate_phy_addr(hw, phy_addr)) {
                                hw->phy.addr = phy_addr;
                                ixgbe_get_phy_id(hw);
                                hw->phy.type =
                                        ixgbe_get_phy_type_from_id(hw->phy.id);

                                if (hw->phy.type == ixgbe_phy_unknown) {
                                        hw->phy.ops.read_reg(hw,
                                                  IXGBE_MDIO_PHY_EXT_ABILITY,
                                                  IXGBE_MDIO_PMA_PMD_DEV_TYPE,
                                                  &ext_ability);
                                        if (ext_ability &
                                            (IXGBE_MDIO_PHY_10GBASET_ABILITY |
                                             IXGBE_MDIO_PHY_1000BASET_ABILITY))
                                                hw->phy.type =
                                                         ixgbe_phy_cu_unknown;
                                        else
                                                hw->phy.type =
                                                         ixgbe_phy_generic;
                                }

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
                status = ixgbe_identify_sfp_module_generic(hw);
                break;

        default:
                hw->phy.sfp_type = ixgbe_sfp_type_not_present;
                status = IXGBE_ERR_SFP_NOT_PRESENT;
                break;
        }

        return status;
}

s32 ixgbe_identify_sfp_module_generic(struct ixgbe_hw *hw)
{
        s32 status = IXGBE_ERR_PHY_ADDR_INVALID;
        u8 identifier = 0;
        u8 comp_codes_10g = 0;
        u8 cable_tech = 0;
        u8 cable_spec = 0;

        if (hw->mac.ops.get_media_type(hw) != ixgbe_media_type_fiber) {
                hw->phy.sfp_type = ixgbe_sfp_type_not_present;
                status = IXGBE_ERR_SFP_NOT_PRESENT;
                goto out;
        }

        status = hw->phy.ops.read_i2c_eeprom(hw,
                                             IXGBE_SFF_IDENTIFIER,
                                             &identifier);

        if (status != 0)
                goto err_read_i2c_eeprom;

        /* LAN ID is needed for sfp_type determination */
        hw->mac.ops.set_lan_id(hw);

        if (identifier != IXGBE_SFF_IDENTIFIER_SFP) {
                hw->phy.type = ixgbe_phy_sfp_unsupported;
                status = IXGBE_ERR_SFP_NOT_SUPPORTED;
        } else {
                status = hw->phy.ops.read_i2c_eeprom(hw,
                                                     IXGBE_SFF_10GBE_COMP_CODES,
                                                     &comp_codes_10g);

                if (status != 0)
                        goto err_read_i2c_eeprom;
                status = hw->phy.ops.read_i2c_eeprom(hw,
                                                     IXGBE_SFF_CABLE_TECHNOLOGY,
                                                     &cable_tech);

                if (status != 0)
                        goto err_read_i2c_eeprom;

                if (hw->mac.type == ixgbe_mac_82599EB) {
                        if (cable_tech & IXGBE_SFF_DA_PASSIVE_CABLE) {
                                if (hw->bus.lan_id == 0)
                                        hw->phy.sfp_type =
                                                     ixgbe_sfp_type_da_cu_core0;
                                else
                                        hw->phy.sfp_type =
                                                     ixgbe_sfp_type_da_cu_core1;
                        } else if (cable_tech & IXGBE_SFF_DA_ACTIVE_CABLE) {
                                hw->phy.ops.read_i2c_eeprom(
                                                hw, IXGBE_SFF_CABLE_SPEC_COMP,
                                                &cable_spec);
                                if (cable_spec &
                                    IXGBE_SFF_DA_SPEC_ACTIVE_LIMITING) {
                                        if (hw->bus.lan_id == 0)
                                                hw->phy.sfp_type =
                                                ixgbe_sfp_type_da_act_lmt_core0;
                                        else
                                                hw->phy.sfp_type =
                                                ixgbe_sfp_type_da_act_lmt_core1;
                                } else {
                                        hw->phy.sfp_type =
                                                        ixgbe_sfp_type_unknown;
                                }
                        } else if (comp_codes_10g &
                                   (IXGBE_SFF_10GBASESR_CAPABLE |
                                    IXGBE_SFF_10GBASELR_CAPABLE)) {
                                if (hw->bus.lan_id == 0)
                                        hw->phy.sfp_type =
                                                      ixgbe_sfp_type_srlr_core0;
                                else
                                        hw->phy.sfp_type =
                                                      ixgbe_sfp_type_srlr_core1;
                        } else {
                                hw->phy.sfp_type = ixgbe_sfp_type_unknown;
                        }
                }
        }

out:
        return status;

err_read_i2c_eeprom:
        hw->phy.sfp_type = ixgbe_sfp_type_not_present;
        if (hw->phy.type != ixgbe_phy_nl) {
                hw->phy.id = 0;
                hw->phy.type = ixgbe_phy_unknown;
        }
        return IXGBE_ERR_SFP_NOT_PRESENT;
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

        /*
         * Limiting active cables and 1G Phys must be initialized as
         * SR modules
         */
        if (sfp_type == ixgbe_sfp_type_da_act_lmt_core0 ||
            sfp_type == ixgbe_sfp_type_1g_lx_core0 ||
            sfp_type == ixgbe_sfp_type_1g_cu_core0 ||
            sfp_type == ixgbe_sfp_type_1g_sx_core0)
                sfp_type = ixgbe_sfp_type_srlr_core0;
        else if (sfp_type == ixgbe_sfp_type_da_act_lmt_core1 ||
                 sfp_type == ixgbe_sfp_type_1g_lx_core1 ||
                 sfp_type == ixgbe_sfp_type_1g_cu_core1 ||
                 sfp_type == ixgbe_sfp_type_1g_sx_core1)
                sfp_type = ixgbe_sfp_type_srlr_core1;

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

s32 ixgbe_get_copper_link_capabilities_generic(struct ixgbe_hw *hw,
                                               ixgbe_link_speed *speed,
                                               bool *autoneg)
{
        s32 status;
        u16 speed_ability;

        *speed = 0;
        *autoneg = true;

        status = hw->phy.ops.read_reg(hw, IXGBE_MDIO_PHY_SPEED_ABILITY,
                                      IXGBE_MDIO_PMA_PMD_DEV_TYPE,
                                      &speed_ability);

        if (status == 0) {
                if (speed_ability & IXGBE_MDIO_PHY_SPEED_10G)
                        *speed |= IXGBE_LINK_SPEED_10GB_FULL;
                if (speed_ability & IXGBE_MDIO_PHY_SPEED_1G)
                        *speed |= IXGBE_LINK_SPEED_1GB_FULL;
                if (speed_ability & IXGBE_MDIO_PHY_SPEED_100M)
                        *speed |= IXGBE_LINK_SPEED_100_FULL;
        }

        return status;
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

enum ixgbe_phy_type ixgbe_get_phy_type_from_id(u32 phy_id)
{
        enum ixgbe_phy_type phy_type;

        switch (phy_id) {
        case TN1010_PHY_ID:
                phy_type = ixgbe_phy_tn;
                break;
        case X540_PHY_ID:
                phy_type = ixgbe_phy_aq;
                break;
        case QT2022_PHY_ID:
                phy_type = ixgbe_phy_qt;
                break;
        case ATH_PHY_ID:
                phy_type = ixgbe_phy_nl;
                break;
        default:
                phy_type = ixgbe_phy_unknown;
                break;
        }

        return phy_type;
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

s32 ixgbe_read_i2c_eeprom_generic(struct ixgbe_hw *hw, u8 byte_offset,
                                  u8 *eeprom_data)
{
        return hw->phy.ops.read_i2c_byte(hw, byte_offset,
                                         IXGBE_I2C_EEPROM_DEV_ADDR,
                                         eeprom_data);
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

s32 ixgbe_read_i2c_byte_generic(struct ixgbe_hw *hw, u8 byte_offset,
                                u8 dev_addr, u8 *data)
{
        s32 status = 0;
        u32 max_retry = 10;
        u32 retry = 0;
        u16 swfw_mask = 0;
        bool nack = 1;
        *data = 0;

        if (IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_LAN_ID_1)
                swfw_mask = IXGBE_GSSR_PHY1_SM;
        else
                swfw_mask = IXGBE_GSSR_PHY0_SM;

        do {
                if (hw->mac.ops.acquire_swfw_sync(hw, swfw_mask)
                    != 0) {
                        status = IXGBE_ERR_SWFW_SYNC;
                        goto read_byte_out;
                }

                ixgbe_i2c_start(hw);

                /* Device Address and write indication */
                status = ixgbe_clock_out_i2c_byte(hw, dev_addr);
                if (status != 0)
                        goto fail;

                status = ixgbe_get_i2c_ack(hw);
                if (status != 0)
                        goto fail;

                status = ixgbe_clock_out_i2c_byte(hw, byte_offset);
                if (status != 0)
                        goto fail;

                status = ixgbe_get_i2c_ack(hw);
                if (status != 0)
                        goto fail;

                ixgbe_i2c_start(hw);

                /* Device Address and read indication */
                status = ixgbe_clock_out_i2c_byte(hw, (dev_addr | 0x1));
                if (status != 0)
                        goto fail;

                status = ixgbe_get_i2c_ack(hw);
                if (status != 0)
                        goto fail;

                status = ixgbe_clock_in_i2c_byte(hw, data);
                if (status != 0)
                        goto fail;

                status = ixgbe_clock_out_i2c_bit(hw, nack);
                if (status != 0)
                        goto fail;

                ixgbe_i2c_stop(hw);
                break;

fail:
                ixgbe_i2c_bus_clear(hw);
                hw->mac.ops.release_swfw_sync(hw, swfw_mask);
                msleep(100);
                retry++;
        } while (retry < max_retry);

        hw->mac.ops.release_swfw_sync(hw, swfw_mask);

read_byte_out:
        return status;
}

static void ixgbe_i2c_start(struct ixgbe_hw *hw)
{
        u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);

        /* Start condition must begin with data and clock high */
        ixgbe_set_i2c_data(hw, &i2cctl, 1);
        ixgbe_raise_i2c_clk(hw, &i2cctl);

        /* Setup time for start condition (4.7us) */
        udelay(IXGBE_I2C_T_SU_STA);

        ixgbe_set_i2c_data(hw, &i2cctl, 0);

        /* Hold time for start condition (4us) */
        udelay(IXGBE_I2C_T_HD_STA);

        ixgbe_lower_i2c_clk(hw, &i2cctl);

        /* Minimum low period of clock is 4.7 us */
        udelay(IXGBE_I2C_T_LOW);

}

static void ixgbe_i2c_stop(struct ixgbe_hw *hw)
{
        u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);

        /* Stop condition must begin with data low and clock high */
        ixgbe_set_i2c_data(hw, &i2cctl, 0);
        ixgbe_raise_i2c_clk(hw, &i2cctl);

        /* Setup time for stop condition (4us) */
        udelay(IXGBE_I2C_T_SU_STO);

        ixgbe_set_i2c_data(hw, &i2cctl, 1);

        /* bus free time between stop and start (4.7us)*/
        udelay(IXGBE_I2C_T_BUF);
}

static s32 ixgbe_clock_in_i2c_byte(struct ixgbe_hw *hw, u8 *data)
{
        s32 i;
        bool bit = 0;

        for (i = 7; i >= 0; i--) {
                ixgbe_clock_in_i2c_bit(hw, &bit);
                *data |= bit << i;
        }

        return 0;
}

static s32 ixgbe_clock_out_i2c_byte(struct ixgbe_hw *hw, u8 data)
{
        s32 status = 0;
        s32 i;
        u32 i2cctl;
        bool bit;

        for (i = 7; i >= 0; i--) {
                bit = (data >> i) & 0x1;
                status = ixgbe_clock_out_i2c_bit(hw, bit);

                if (status != 0)
                        break;
        }

        /* Release SDA line (set high) */
        i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);
        i2cctl |= IXGBE_I2C_DATA_OUT;
        IXGBE_WRITE_REG(hw, IXGBE_I2CCTL, i2cctl);
        IXGBE_WRITE_FLUSH(hw);

        return status;
}

static s32 ixgbe_get_i2c_ack(struct ixgbe_hw *hw)
{
        s32 status = 0;
        u32 i = 0;
        u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);
        u32 timeout = 10;
        bool ack = 1;

        ixgbe_raise_i2c_clk(hw, &i2cctl);


        /* Minimum high period of clock is 4us */
        udelay(IXGBE_I2C_T_HIGH);

        /* Poll for ACK.  Note that ACK in I2C spec is
         * transition from 1 to 0 */
        for (i = 0; i < timeout; i++) {
                i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);
                ack = ixgbe_get_i2c_data(&i2cctl);

                udelay(1);
                if (ack == 0)
                        break;
        }

        if (ack == 1) {
                status = IXGBE_ERR_I2C;
        }

        ixgbe_lower_i2c_clk(hw, &i2cctl);

        /* Minimum low period of clock is 4.7 us */
        udelay(IXGBE_I2C_T_LOW);

        return status;
}

static void ixgbe_raise_i2c_clk(struct ixgbe_hw *hw, u32 *i2cctl)
{
        u32 i = 0;
        u32 timeout = IXGBE_I2C_CLOCK_STRETCHING_TIMEOUT;
        u32 i2cctl_r = 0;

        for (i = 0; i < timeout; i++) {
                *i2cctl |= IXGBE_I2C_CLK_OUT;

                IXGBE_WRITE_REG(hw, IXGBE_I2CCTL, *i2cctl);
                IXGBE_WRITE_FLUSH(hw);
                /* SCL rise time (1000ns) */
                udelay(IXGBE_I2C_T_RISE);

                i2cctl_r = IXGBE_READ_REG(hw, IXGBE_I2CCTL);
                if (i2cctl_r & IXGBE_I2C_CLK_IN)
                        break;
        }
}

static void ixgbe_lower_i2c_clk(struct ixgbe_hw *hw, u32 *i2cctl)
{

        *i2cctl &= ~IXGBE_I2C_CLK_OUT;

        IXGBE_WRITE_REG(hw, IXGBE_I2CCTL, *i2cctl);
        IXGBE_WRITE_FLUSH(hw);

        /* SCL fall time (300ns) */
        udelay(IXGBE_I2C_T_FALL);
}

static s32 ixgbe_clock_in_i2c_bit(struct ixgbe_hw *hw, bool *data)
{
        u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);

        ixgbe_raise_i2c_clk(hw, &i2cctl);

        /* Minimum high period of clock is 4us */
        udelay(IXGBE_I2C_T_HIGH);

        i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);
        *data = ixgbe_get_i2c_data(&i2cctl);

        ixgbe_lower_i2c_clk(hw, &i2cctl);

        /* Minimum low period of clock is 4.7 us */
        udelay(IXGBE_I2C_T_LOW);

        return 0;
}

static s32 ixgbe_clock_out_i2c_bit(struct ixgbe_hw *hw, bool data)
{
        s32 status;
        u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);

        status = ixgbe_set_i2c_data(hw, &i2cctl, data);
        if (status == 0) {
                ixgbe_raise_i2c_clk(hw, &i2cctl);

                /* Minimum high period of clock is 4us */
                udelay(IXGBE_I2C_T_HIGH);

                ixgbe_lower_i2c_clk(hw, &i2cctl);

                /* Minimum low period of clock is 4.7 us.
                 * This also takes care of the data hold time.
                 */
                udelay(IXGBE_I2C_T_LOW);
        } else {
                status = IXGBE_ERR_I2C;
        }

        return status;
}

static s32 ixgbe_set_i2c_data(struct ixgbe_hw *hw, u32 *i2cctl, bool data)
{
        s32 status = 0;

        if (data)
                *i2cctl |= IXGBE_I2C_DATA_OUT;
        else
                *i2cctl &= ~IXGBE_I2C_DATA_OUT;

        IXGBE_WRITE_REG(hw, IXGBE_I2CCTL, *i2cctl);
        IXGBE_WRITE_FLUSH(hw);

        /* Data rise/fall (1000ns/300ns) and set-up time (250ns) */
        udelay(IXGBE_I2C_T_RISE + IXGBE_I2C_T_FALL + IXGBE_I2C_T_SU_DATA);

        /* Verify data was set correctly */
        *i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);
        if (data != ixgbe_get_i2c_data(i2cctl)) {
                status = IXGBE_ERR_I2C;
        }

        return status;
}

static bool ixgbe_get_i2c_data(u32 *i2cctl)
{
        bool data;

        if (*i2cctl & IXGBE_I2C_DATA_IN)
                data = 1;
        else
                data = 0;

        return data;
}

static void ixgbe_i2c_bus_clear(struct ixgbe_hw *hw)
{
        u32 i2cctl = IXGBE_READ_REG(hw, IXGBE_I2CCTL);
        u32 i;

        ixgbe_i2c_start(hw);

        ixgbe_set_i2c_data(hw, &i2cctl, 1);

        for (i = 0; i < 9; i++) {
                ixgbe_raise_i2c_clk(hw, &i2cctl);

                /* Min high period of clock is 4us */
                udelay(IXGBE_I2C_T_HIGH);

                ixgbe_lower_i2c_clk(hw, &i2cctl);

                /* Min low period of clock is 4.7us*/
                udelay(IXGBE_I2C_T_LOW);
        }

        ixgbe_i2c_start(hw);

        /* Put the i2c bus back to default state */
        ixgbe_i2c_stop(hw);
}
