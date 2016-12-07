#ifndef _PP2_PRINT_H_
#define _PP2_PRINT_H_

#define PP2_ERR  (1)
#define PP2_WARN (2)
#define PP2_INFO (3)
#define PP2_DBG  (4)

#define PP2_INFO_LEVEL (PP2_INFO)

#define pp2_print(_level, ...) \
do { \
	if ((_level) <= (PP2_INFO_LEVEL)) { \
		printf(__VA_ARGS__); \
	} \
} while (0)

#define dbg(_func, ...) \
do { \
	if ((PP2_DBG) <= (PP2_INFO_LEVEL)) { \
		_func(__VA_ARGS__); \
	} \
} while (0)

#define pp2_err(...) \
	pp2_print(PP2_ERR, "[ERR] " __VA_ARGS__)

#define pp2_warn(...) \
	pp2_print(PP2_WARN, "[WARN] " __VA_ARGS__)

#define pp2_info(...) \
	pp2_print(PP2_INFO, "[INFO] " __VA_ARGS__)

#define pp2_dbg(...) \
	pp2_print(PP2_DBG, "[DBG] " __VA_ARGS__)

#define pp2_info_fmt(fmt, ...) \
	pp2_info("%24s : %3d : " fmt "\n", \
		__func__, __LINE__, ##__VA_ARGS__) \

#define pp2_dbg_fmt(fmt, ...) \
	pp2_dbg("%24s : %3d : " fmt "\n", \
	       __func__, __LINE__, ##__VA_ARGS__) \

#endif /* _PP2_PRINT_H_ */
