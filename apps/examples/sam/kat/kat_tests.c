
#include <stdlib.h>
#include <string.h>

#include "mv_sam.h"

static struct sam_cio *cio_hndl;

int main(int argc, char **argv)
{
	struct sam_cio_params cio_params;

	cio_params.id = 0;
	cio_params.size = 32;
	cio_params.num_sessions = 256;
	cio_params.max_buf_size = 2048;

	if (sam_cio_init(&cio_params, &cio_hndl)) {
		printf("%s: initialization failed\n", argv[0]);
		return 1;
	}
	printf("%s successfully started\n", argv[0]);

	if (sam_cio_deinit(cio_hndl)) {
		printf("%s: un-initialization failed\n", argv[0]);
		return 1;
	}
	printf("%s successfully finished\n", argv[0]);

	return 0;
}
