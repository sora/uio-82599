/* EEPROM byte offsets */
#define IXGBE_SFF_IDENTIFIER            0x0
#define IXGBE_SFF_IDENTIFIER_SFP        0x3
#define IXGBE_SFF_10GBE_COMP_CODES      0x3
#define IXGBE_SFF_CABLE_TECHNOLOGY      0x8
#define IXGBE_SFF_CABLE_SPEC_COMP       0x3C

/* Bitmasks */
#define IXGBE_SFF_DA_PASSIVE_CABLE      0x4
#define IXGBE_SFF_DA_ACTIVE_CABLE       0x8
#define IXGBE_SFF_DA_SPEC_ACTIVE_LIMITING       0x4
#define IXGBE_SFF_10GBASESR_CAPABLE     0x10
#define IXGBE_SFF_10GBASELR_CAPABLE     0x20

/* Special PHY Init Routine */
#define IXGBE_PHY_INIT_OFFSET_NL        0x002B
#define IXGBE_PHY_INIT_END_NL           0xFFFF

/* PHY IDs*/
#define TN1010_PHY_ID   0x00A19410
#define X540_PHY_ID     0x01540200
#define QT2022_PHY_ID   0x0043A400
#define ATH_PHY_ID      0x03429050

/* Flow control defines */
#define IXGBE_TAF_SYM_PAUSE             0x400
#define IXGBE_TAF_ASM_PAUSE             0x800

s32 ixgbe_reset_phy_generic(struct ixgbe_hw *hw);
s32 ixgbe_identify_phy_generic(struct ixgbe_hw *hw);
s32 ixgbe_get_sfp_init_sequence_offsets(struct ixgbe_hw *hw,
                                        u16 *list_offset,
                                        u16 *data_offset);
s32 ixgbe_identify_module_generic(struct ixgbe_hw *hw);
bool ixgbe_validate_phy_addr(struct ixgbe_hw *hw, u32 phy_addr);
s32 ixgbe_get_phy_id(struct ixgbe_hw *hw);
enum ixgbe_phy_type ixgbe_get_phy_type_from_id(u32 phy_id);
s32 ixgbe_identify_sfp_module_generic(struct ixgbe_hw *hw);
