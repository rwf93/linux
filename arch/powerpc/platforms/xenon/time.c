/*
 *  Xenon time and rtc routines.
 *
 *  Copyright (C) 2010 Herbert Poetzl
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

static int __init xenon_rtc_init(void)
{
	struct platform_device *pdev;

	pdev = platform_device_register_simple("rtc-xenon", -1, NULL, 0);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	return 0;
}

module_init(xenon_rtc_init);