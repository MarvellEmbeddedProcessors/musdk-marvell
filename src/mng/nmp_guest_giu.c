/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"
#include "mng/mv_nmp_guest_giu.h"

#include "lf/mng_cmd_desc.h"

#include "nmp_guest.h"

static int nmp_guest_find_lf_from_match(struct nmp_guest *guest, char *match, u8 *lf_type, u8 *lf_id)
{
	int i;

	for (i = 0; i < guest->total_giu_object_count; i++)
		if (strcmp(match, guest->giu_object[i].match) == 0) {
			*lf_type = guest->giu_object[i].lf_type;
			*lf_id = guest->giu_object[i].lf_id;
			break;
		}
	if (i == guest->total_giu_object_count) {
		pr_err("giu match '%s' not found\n", match);
		return -1;
	}

	return 0;
}

int nmp_guest_giu_gpio_enable(char *gpio_match)
{
	struct nmp_guest *guest = nmp_guest_get_handle();
	struct guest_cmd_resp resp;
	u8 lf_type = 0, lf_id = 0;
	int ret;

	ret = nmp_guest_find_lf_from_match(guest, gpio_match, &lf_type, &lf_id);
	if (ret)
		return ret;

	ret = send_internal_msg(guest, lf_type, lf_id, MSG_F_GUEST_GPIO_ENABLE, 0, NULL, 0, &resp, sizeof(resp));
	if (ret)
		return ret;
	if (resp.status == RESP_STATUS_FAIL)
		return -1;

	return 0;
}

int nmp_guest_giu_gpio_disable(char *gpio_match)
{
	struct nmp_guest *guest = nmp_guest_get_handle();
	struct guest_cmd_resp resp;
	u8 lf_type = 0, lf_id = 0;
	int ret;

	ret = nmp_guest_find_lf_from_match(guest, gpio_match, &lf_type, &lf_id);
	if (ret)
		return ret;

	ret = send_internal_msg(guest, lf_type, lf_id, MSG_F_GUEST_GPIO_DISABLE, 0, NULL, 0, &resp, sizeof(resp));
	if (ret)
		return ret;
	if (resp.status == RESP_STATUS_FAIL)
		return -1;

	return 0;
}

int nmp_guest_giu_gpio_get_link_state(char *gpio_match, int *en)
{
	struct nmp_guest *guest = nmp_guest_get_handle();
	struct guest_cmd_resp resp;
	u8 lf_type = 0, lf_id = 0;
	int ret;

	ret = nmp_guest_find_lf_from_match(guest, gpio_match, &lf_type, &lf_id);
	if (ret)
		return ret;
pr_line;
	ret = send_internal_msg(guest, lf_type, lf_id, MSG_F_GUEST_GPIO_GET_LINK_STATE, 0, NULL, 0,
				&resp, sizeof(resp));
	if (ret)
		return ret;
	if (resp.status == RESP_STATUS_FAIL)
		return -1;

	*en = (int)resp.giu_resp.link_state;

	return 0;
}


