/*
 * check.h — minimal unit-test helpers for the host tests (no framework needed).
 */
#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond) do {                                                     \
    g_checks++;                                                              \
    if (!(cond)) {                                                           \
        g_failures++;                                                        \
        fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
    }                                                                        \
} while (0)

#define CHECK_NEAR(a, b, tol) do {                                           \
    g_checks++;                                                              \
    double _a = (double)(a), _b = (double)(b);                               \
    if (fabs(_a - _b) > (tol)) {                                             \
        g_failures++;                                                        \
        fprintf(stderr, "  FAIL %s:%d: %s = %g, expected %g (+/- %g)\n",     \
                __FILE__, __LINE__, #a, _a, _b, (double)(tol));              \
    }                                                                        \
} while (0)

#define RUN(test) do { printf("  %s\n", #test); test(); } while (0)

static inline int check_summary(const char *suite)
{
    printf("%s: %d checks, %d failures\n", suite, g_checks, g_failures);
    return g_failures ? 1 : 0;
}
