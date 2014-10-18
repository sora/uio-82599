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

        /* Don't reset PHY if it's shut down due to overtemp. */
        if (!hw->phy.reset_if_overtemp &&
            (IXGBE_ERR_OVERTEMP == hw->phy.ops.check_overtemp(hw)))
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
