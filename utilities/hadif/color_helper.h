/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: color helper functions
 */

#ifndef _COLOR_HELPER_H_
#define _COLOR_HELPER_H_

#include <cmath>

/**
 * the file provides 2 palettes:
 *  1) Jet
 *  2) Green-Jet
 */

/** Given an array of colors, a palette is created that linearly interpolates
 * through all the colors. **/
static void color_util_build_color_table(double color_palette[][3],
        int palette_size, float lut[][3], int lut_size)
{
    for (int idx = 0; idx < lut_size; idx++)
    {
        double znorm = ((double)idx) / lut_size;

        int color_index = (palette_size - 1) * znorm;
        double alpha = (palette_size - 1) * znorm - color_index;

        for (int i = 0; i < 3; i++)
        {
            lut[idx][i] = color_palette[color_index][i] * (1.0 - alpha) +
                color_palette[color_index + 1][i] * alpha;
        }
    }
}

#define JET_COLORS_LUT_SIZE 1024
static float jet_colors[JET_COLORS_LUT_SIZE][3];
static int jet_colors_initialized = 0;

static void init_color_table_jet()
{
    double jet[][3] = {{0, 0, 1}, {0, .5, .5}, {.8, .8, 0}, {1, 0, 0}};

    color_util_build_color_table(jet, sizeof(jet) / (sizeof(double) * 3), 
            jet_colors, JET_COLORS_LUT_SIZE);
    jet_colors_initialized = 1;
}

static inline float *color_util_jet(double v)
{
    if (!jet_colors_initialized)
        init_color_table_jet();

    v = fmax(0, v);
    v = fmin(1, v);

    int idx = (JET_COLORS_LUT_SIZE - 1) * v;
    return jet_colors[idx];
}

#define GREEN_JET_COLORS_LUT_SIZE 1024
static float green_jet_colors[GREEN_JET_COLORS_LUT_SIZE][3];
static int green_jet_colors_initialized = 0;

static void init_color_table_green_jet()
{
    double green_jet[][3] = {{0, 1, 0}, {0, .5, .5}, {.8, 0, .8}, {1, 0, 0}};

    color_util_build_color_table(green_jet, sizeof(green_jet) / (sizeof(double) * 3),
            green_jet_colors, GREEN_JET_COLORS_LUT_SIZE);
    green_jet_colors_initialized = 1;
}

static inline float *color_util_green_jet(double v)
{
    if (!green_jet_colors_initialized)
        init_color_table_green_jet();

    v = fmax(0, v);
    v = fmin(1, v);

    int idx = (GREEN_JET_COLORS_LUT_SIZE - 1) * v;
    return green_jet_colors[idx];
}

#endif
