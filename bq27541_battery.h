#ifndef __LINUX_BQ27X00_BATTERY_H__
#define __LINUX_BQ27X00_BATTERY_H__

/**
 * struct bq27000_plaform_data - Platform data for bq27000 devices
 * @name: Name of the battery. If NULL the driver will fallback to "bq27000".
 * @read: HDQ read callback.
 *	This function should provide access to the HDQ bus the battery is
 *	connected to.
 *	The first parameter is a pointer to the battery device, the second the
 *	register to be read. The return value should either be the content of
 *	the passed register or an error value.
 */
#define ATRATE_MA            -100           // USER CONFIG: AtRate setting (mA)
#define I2CSLAVEADDR         0x55           // 7-bit slave address

#define DRIVER_VERSION			"1.2.0"

#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_AR		        0x02
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18 /*FilterCC*/
#define BQ27x00_REG_TTECP		0x26 /**/
#define BQ27x00_REG_NAC			0x0C /* Nominal available capaciy */
#define BQ27x00_REG_LMD			0x12 /* Full Charge Capacity */
#define BQ27x00_REG_CYCT		0x2A /* Cycle count total */
#define BQ27x00_REG_AE			0x22 /* Available enery */

#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_REG_ILMD		0x76 /* Initial last measured discharge */
#define BQ27000_FLAG_CHGS		BIT(7)
#define BQ27000_FLAG_FC			BIT(5)

#define BQ27541_REG_SOC			0x2C
#define BQ27541_REG_DCAP		0x3C /* Design capacity */
#define BQ27541_REG_UFSOC		0x04 /* Unfilter SOC */
#define BQ27541_REG_FAC			0x0e /* Full Available Power */

#define BQ27541_REG_RM			0x10 /* Remaining Capacity */
#define BQ27541_REG_SI			0x1A /* Standby Current */
#define BQ27541_REG_UFFCC		0x1C /* Unfilter Full Charge Power*/

#define BQ27541_REG_MLI			0x1E /* Maxload Current */
#define BQ27541_REG_UFRM		0x20 /* Unfilter Remaining Current*/
#define BQ27541_REG_FRM			0x22 /* filter Remaining Current */
#define BQ27541_REG_AP			0x24 /* Average Power*/

#define BQ27541_REG_SOH			0x2E /* State of Health*/
#define BQ27541_REG_PCHG		0x34 /* PassedCharge*/
#define BQ27541_REG_DOD0		0x36 /* DOD0*/
#define BQ27541_REG_DFCLS		0x3E /* SelfDischargeCurrent */
#define BQ27541_REG_DFBLK		0x3F /* SelfDischargeCurrent */
#define BQ27541_REG_ADF		        0x40 /* SelfDischargeCurrent */
#define BQ27541_REG_ACKSDFD             0x54
#define BQ27541_REG_INTTEP		0x28 /* Internal Temperture */
#define BQ27541_REG_SDSG		0x38 /* SelfDischargeCurrent */
#define BQ27541_REG_DFDCKS		0x60 /* SelfDischargeCurrent */
#define BQ27541_REG_DFDCNTL		0x61 /* SelfDischargeCurrent */
#define BQ27541_REG_DNAMELEN		0x62 /* SelfDischargeCurrent */
#define BQ27541_REG_DNAME		0x63 /* SelfDischargeCurrent */
#define BQ27500_FLAG_DSG		BIT(0)
#define BQ27500_FLAG_FC			BIT(9)

#define BQ27000_RS			20 /* Resistor sense */


struct bq27000_platform_data {
	const char *name;
	int (*read)(struct device *dev, unsigned int);
};

#endif
