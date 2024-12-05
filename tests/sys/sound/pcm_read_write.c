/*-
 * Copyright (c) 2025 Florian Walpen
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * These tests exercise conversion functions of the sound module, used to read
 * pcm samples from a buffer, and write pcm samples to a buffer. The test cases
 * are non-exhaustive, but should detect systematic errors in conversion of the
 * various sample formats supported. In particular, the test cases establish
 * correctness independent of the machine's endianness, making them suitable to
 * check for architecture-specific problems.
 */

#include <sys/types.h>
#include <sys/soundcard.h>

#include <atf-c.h>
#include <stdio.h>
#include <string.h>

#include <dev/sound/pcm/sound.h>
#include <dev/sound/pcm/pcm.h>
#include <dev/sound/pcm/g711.h>

/* Generic test data, with buffer content matching the sample values. */
struct afmt_test_data {
	uint8_t buffer[4];
	size_t size;
	int format;
	intpcm_t value;
	intpcm_t shifted;
} static const afmt_tests[] = {
	/* 8 bit sample formats. */
	{{0x01, 0x00, 0x00, 0x00}, 1, AFMT_S8, 0x00000001, 0x01000000},
	{{0x81, 0x00, 0x00, 0x00}, 1, AFMT_S8, 0xffffff81, 0x81000000},
	{{0x01, 0x00, 0x00, 0x00}, 1, AFMT_U8, 0xffffff81, 0x81000000},
	{{0x81, 0x00, 0x00, 0x00}, 1, AFMT_U8, 0x00000001, 0x01000000},

	/* 16 bit sample formats. */
	{{0x01, 0x02, 0x00, 0x00}, 2, AFMT_S16_LE, 0x00000201, 0x02010000},
	{{0x81, 0x82, 0x00, 0x00}, 2, AFMT_S16_LE, 0xffff8281, 0x82810000},
	{{0x01, 0x02, 0x00, 0x00}, 2, AFMT_S16_BE, 0x00000102, 0x01020000},
	{{0x81, 0x82, 0x00, 0x00}, 2, AFMT_S16_BE, 0xffff8182, 0x81820000},
	{{0x01, 0x02, 0x00, 0x00}, 2, AFMT_U16_LE, 0xffff8201, 0x82010000},
	{{0x81, 0x82, 0x00, 0x00}, 2, AFMT_U16_LE, 0x00000281, 0x02810000},
	{{0x01, 0x02, 0x00, 0x00}, 2, AFMT_U16_BE, 0xffff8102, 0x81020000},
	{{0x81, 0x82, 0x00, 0x00}, 2, AFMT_U16_BE, 0x00000182, 0x01820000},

	/* 24 bit sample formats. */
	{{0x01, 0x02, 0x03, 0x00}, 3, AFMT_S24_LE, 0x00030201, 0x03020100},
	{{0x81, 0x82, 0x83, 0x00}, 3, AFMT_S24_LE, 0xff838281, 0x83828100},
	{{0x01, 0x02, 0x03, 0x00}, 3, AFMT_S24_BE, 0x00010203, 0x01020300},
	{{0x81, 0x82, 0x83, 0x00}, 3, AFMT_S24_BE, 0xff818283, 0x81828300},
	{{0x01, 0x02, 0x03, 0x00}, 3, AFMT_U24_LE, 0xff830201, 0x83020100},
	{{0x81, 0x82, 0x83, 0x00}, 3, AFMT_U24_LE, 0x00038281, 0x03828100},
	{{0x01, 0x02, 0x03, 0x00}, 3, AFMT_U24_BE, 0xff810203, 0x81020300},
	{{0x81, 0x82, 0x83, 0x00}, 3, AFMT_U24_BE, 0x00018283, 0x01828300},

	/* 32 bit sample formats. */
	{{0x01, 0x02, 0x03, 0x04}, 4, AFMT_S32_LE, 0x04030201, 0x04030201},
	{{0x81, 0x82, 0x83, 0x84}, 4, AFMT_S32_LE, 0x84838281, 0x84838281},
	{{0x01, 0x02, 0x03, 0x04}, 4, AFMT_S32_BE, 0x01020304, 0x01020304},
	{{0x81, 0x82, 0x83, 0x84}, 4, AFMT_S32_BE, 0x81828384, 0x81828384},
	{{0x01, 0x02, 0x03, 0x04}, 4, AFMT_U32_LE, 0x84030201, 0x84030201},
	{{0x81, 0x82, 0x83, 0x84}, 4, AFMT_U32_LE, 0x04838281, 0x04838281},
	{{0x01, 0x02, 0x03, 0x04}, 4, AFMT_U32_BE, 0x81020304, 0x81020304},
	{{0x81, 0x82, 0x83, 0x84}, 4, AFMT_U32_BE, 0x01828384, 0x01828384},

	/* u-law and A-law sample formats. */
	{{0x01, 0x00, 0x00, 0x00}, 1, AFMT_MU_LAW, 0xffffff87, 0x87000000},
	{{0x81, 0x00, 0x00, 0x00}, 1, AFMT_MU_LAW, 0x00000079, 0x79000000},
	{{0x2a, 0x00, 0x00, 0x00}, 1, AFMT_A_LAW, 0xffffff83, 0x83000000},
	{{0xab, 0x00, 0x00, 0x00}, 1, AFMT_A_LAW, 0x00000079, 0x79000000}
};

ATF_TC(pcm_read);
ATF_TC_HEAD(pcm_read, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "Read and verify different pcm sample formats.");
}
ATF_TC_BODY(pcm_read, tc)
{
	const struct afmt_test_data *test;
	uint8_t src[4];
	intpcm_t result;
	size_t i;

	for (i = 0; i < nitems(afmt_tests); i++) {
		test = &afmt_tests[i];

		/* Copy byte representation, fill with distinctive pattern. */
		memset(src, 0x66, sizeof(src));
		memcpy(src, test->buffer, test->size);

		/* Read sample at format magnitude. */
		result = pcm_sample_read(src, test->format);
		ATF_CHECK_MSG(result == test->value,
		    "read_test[%zu].value: expected=0x%08x, result=0x%08x",
		    i, test->value, result);

		/* Read sample at full 32 bit magnitude. */
		result = pcm_sample_read_shift(src, test->format);
		ATF_CHECK_MSG(result == test->shifted,
		    "read_test[%zu].shifted: expected=0x%08x, result=0x%08x",
		    i, test->shifted, result);
	}
}

ATF_TC(pcm_write);
ATF_TC_HEAD(pcm_write, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "Write and verify different pcm sample formats.");
}
ATF_TC_BODY(pcm_write, tc)
{
	const struct afmt_test_data *test;
	uint8_t dst[4];
	size_t i;

	for (i = 0; i < nitems(afmt_tests); i++) {
		test = &afmt_tests[i];

		/* Write sample of format specific magnitude. */
		memset(dst, 0x00, sizeof(dst));
		pcm_sample_write(dst, test->value, test->format);
		ATF_CHECK_MSG(memcmp(dst, test->buffer, sizeof(dst)) == 0,
		    "write_test[%zu].value: expected=0x%08x, result=0x%08x",
		    i, *(const uint32_t *)test->buffer, *(uint32_t *)dst);

		/* Write normalized sample of full 32 bit magnitude. */
		memset(dst, 0x00, sizeof(dst));
		pcm_sample_write_shift(dst, test->shifted, test->format);
		ATF_CHECK_MSG(memcmp(dst, test->buffer, sizeof(dst)) == 0,
		    "write_test[%zu].value: expected=0x%08x, result=0x%08x",
		    i, *(const uint32_t *)test->buffer, *(uint32_t *)dst);
	}
}

ATF_TC(pcm_format_bits);
ATF_TC_HEAD(pcm_format_bits, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "Verify bit width of different pcm sample formats.");
}
ATF_TC_BODY(pcm_format_bits, tc)
{
	const struct afmt_test_data *test;
	size_t bits;
	size_t i;

	for (i = 0; i < nitems(afmt_tests); i++) {
		test = &afmt_tests[i];

		/* Check bit width determined for given sample format. */
		bits = AFMT_BIT(test->format);
		ATF_CHECK_MSG(bits == test->size * 8,
		    "format_bits[%zu].size: expected=%zu, result=%zu",
		    i, test->size * 8, bits);
	}
}

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, pcm_read);
	ATF_TP_ADD_TC(tp, pcm_write);
	ATF_TP_ADD_TC(tp, pcm_format_bits);

	return atf_no_error();
}
