/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __GEN_H__
#define __GEN_H__

/**
 * TODO
 *
 * @param[in]	_type	TODO
 * @param[in]	_member	TODO
 *
 * @return	TODO
 */
#define MEMBER_OFFSET(_type, _member) (PTR_TO_UINT(&((_type *)0)->_member))

#if !(defined(ARRAY_SIZE))
/**
 * TODO
 *
 * @param[in]	_arr	TODO
 *
 * @return	TODO
 */
#define ARRAY_SIZE(_arr)   (sizeof(_arr) / sizeof((_arr)[0]))
#endif /* !defined(ARRAY_SIZE) */

#endif /* __GEN_H__ */
