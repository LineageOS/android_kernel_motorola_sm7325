// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__
#include "msm_drv.h"
#include "sde_dbg.h"

#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include "msm_cooling_device.h"
#include <linux/backlight.h>
#include <linux/string.h>
#include "dsi_drm.h"
#include "dsi_display.h"
#include "sde_crtc.h"
#include "sde_rm.h"
#include "sde_vm.h"
#include <drm/drm_probe_helper.h>

#define BL_NODE_NAME_SIZE 32
#define HDR10_PLUS_VSIF_TYPE_CODE      0x81

/* Autorefresh will occur after FRAME_CNT frames. Large values are unlikely */
#define AUTOREFRESH_MAX_FRAME_CNT 6

#define SDE_DEBUG_CONN(c, fmt, ...) SDE_DEBUG("conn%d " fmt,\
		(c) ? (c)->base.base.id : -1, ##__VA_ARGS__)

#define SDE_ERROR_CONN(c, fmt, ...) SDE_ERROR("conn%d " fmt,\
		(c) ? (c)->base.base.id : -1, ##__VA_ARGS__)

static const struct drm_prop_enum_list e_topology_name[] = {
	{SDE_RM_TOPOLOGY_NONE,	"sde_none"},
	{SDE_RM_TOPOLOGY_SINGLEPIPE,	"sde_singlepipe"},
	{SDE_RM_TOPOLOGY_SINGLEPIPE_DSC,	"sde_singlepipe_dsc"},
	{SDE_RM_TOPOLOGY_SINGLEPIPE_VDC,	"sde_singlepipe_vdc"},
	{SDE_RM_TOPOLOGY_DUALPIPE,	"sde_dualpipe"},
	{SDE_RM_TOPOLOGY_DUALPIPE_DSC,	"sde_dualpipe_dsc"},
	{SDE_RM_TOPOLOGY_DUALPIPE_3DMERGE,	"sde_dualpipemerge"},
	{SDE_RM_TOPOLOGY_DUALPIPE_3DMERGE_DSC,	"sde_dualpipemerge_dsc"},
	{SDE_RM_TOPOLOGY_DUALPIPE_3DMERGE_VDC,	"sde_dualpipemerge_vdc"},
	{SDE_RM_TOPOLOGY_DUALPIPE_DSCMERGE,	"sde_dualpipe_dscmerge"},
	{SDE_RM_TOPOLOGY_PPSPLIT,	"sde_ppsplit"},
	{SDE_RM_TOPOLOGY_QUADPIPE_3DMERGE,	"sde_quadpipemerge"},
	{SDE_RM_TOPOLOGY_QUADPIPE_3DMERGE_DSC,	"sde_quadpipe_3dmerge_dsc"},
	{SDE_RM_TOPOLOGY_QUADPIPE_DSCMERGE,	"sde_quadpipe_dscmerge"},
	{SDE_RM_TOPOLOGY_QUADPIPE_DSC4HSMERGE,	"sde_quadpipe_dsc4hsmerge"},
};
static const struct drm_prop_enum_list e_topology_control[] = {
	{SDE_RM_TOPCTL_RESERVE_LOCK,	"reserve_lock"},
	{SDE_RM_TOPCTL_RESERVE_CLEAR,	"reserve_clear"},
	{SDE_RM_TOPCTL_DSPP,		"dspp"},
	{SDE_RM_TOPCTL_DS,		"ds"},
};
static const struct drm_prop_enum_list e_power_mode[] = {
	{SDE_MODE_DPMS_ON,	"ON"},
	{SDE_MODE_DPMS_LP1,	"LP1"},
	{SDE_MODE_DPMS_LP2,	"LP2"},
	{SDE_MODE_DPMS_OFF,	"OFF"},
};
static const struct drm_prop_enum_list e_qsync_mode[] = {
	{SDE_RM_QSYNC_DISABLED,	"none"},
	{SDE_RM_QSYNC_CONTINUOUS_MODE,	"continuous"},
	{SDE_RM_QSYNC_ONE_SHOT_MODE,	"one_shot"},
};
static const struct drm_prop_enum_list e_frame_trigger_mode[] = {
	{FRAME_DONE_WAIT_DEFAULT, "default"},
	{FRAME_DONE_WAIT_SERIALIZE, "serialize_frame_trigger"},
	{FRAME_DONE_WAIT_POSTED_START, "posted_start"},
};

static const int bl_lut[] = {0, 11, 16, 20, 23, 25, 28, 30, 32, 34, 36, 38, 39, 41, 42, 60, 85, 104, 120, 134, 147, 159, 170, 180, 190, 199, 208, 216, 224, 232, 240, 247, 254, 261, 268, 275, 281, 288, 294, 300, 306, 312, 317, 323, 328, 334, 339, 344, 350, 355, 360, 365, 370, 374, 379, 384, 389, 393, 398, 402, 407, 411, 415, 420, 424, 428, 432, 437, 441, 445, 449, 453, 457, 461, 464, 468, 472, 476, 480, 483, 487, 491, 494, 498, 502, 505, 509, 512, 516, 519, 523, 526, 530, 533, 536, 540, 543, 546, 550, 553, 556, 559, 562, 566, 569, 572, 575, 578, 581, 584, 587, 591, 594, 597, 600, 603, 606, 609, 611, 614, 617, 620, 623, 626, 629, 632, 635, 637, 640, 643, 646, 649, 651, 654, 657, 660, 662, 665, 668, 670, 673, 676, 678, 681, 684, 686, 689, 691, 694, 697, 699, 702, 704, 707, 709, 712, 714, 717, 720, 722, 724, 727, 729, 732, 734, 737, 739, 742, 744, 746, 749, 751, 754, 756, 758, 761, 763, 766, 768, 770, 773, 775, 777, 779, 782, 784, 786, 789, 791, 793, 795, 798, 800, 802, 804, 807, 809, 811, 813, 816, 818, 820, 822, 824, 826, 829, 831, 833, 835, 837, 839, 842, 844, 846, 848, 850, 852, 854, 856, 858, 861, 863, 865, 867, 869, 871, 873, 875, 877, 879, 881, 883, 885, 887, 889, 891, 893, 895, 897, 899, 901, 903, 905, 907, 909, 911, 913, 915, 917, 919, 921, 923, 925, 927, 929, 931, 933, 935, 937, 939, 940, 942, 944, 946, 948, 950, 952, 954, 956, 957, 959, 961, 963, 965, 967, 969, 971, 972, 974, 976, 978, 980, 982, 983, 985, 987, 989, 991, 993, 994, 996, 998, 1000, 1002, 1003, 1005, 1007, 1009, 1010, 1012, 1014, 1016, 1018, 1019, 1021, 1023, 1025, 1030, 1034, 1038, 1042, 1046, 1051, 1055, 1059, 1063, 1067, 1071, 1075, 1079, 1083, 1087, 1091, 1095, 1099, 1103, 1107, 1111, 1115, 1118, 1122, 1126, 1130, 1134, 1137, 1141, 1145, 1149, 1152, 1156, 1160, 1163, 1167, 1171, 1174, 1178, 1181, 1185, 1189, 1192, 1196, 1199, 1203, 1206, 1210, 1213, 1216, 1220, 1223, 1227, 1230, 1234, 1237, 1240, 1244, 1247, 1250, 1254, 1257, 1260, 1263, 1267, 1270, 1273, 1276, 1280, 1283, 1286, 1289, 1292, 1295, 1299, 1302, 1305, 1308, 1311, 1314, 1317, 1320, 1323, 1326, 1329, 1332, 1335, 1338, 1341, 1344, 1347, 1350, 1353, 1356, 1359, 1362, 1365, 1368, 1371, 1374, 1377, 1379, 1382, 1385, 1388, 1391, 1394, 1396, 1399, 1402, 1405, 1408, 1410, 1413, 1416, 1419, 1421, 1424, 1427, 1430, 1432, 1435, 1438, 1440, 1443, 1446, 1448, 1451, 1454, 1456, 1459, 1461, 1464, 1467, 1469, 1472, 1474, 1477, 1480, 1482, 1485, 1487, 1490, 1492, 1495, 1497, 1500, 1502, 1505, 1507, 1510, 1512, 1515, 1517, 1520, 1522, 1524, 1527, 1529, 1532, 1534, 1537, 1539, 1541, 1544, 1546, 1548, 1551, 1553, 1556, 1558, 1560, 1563, 1565, 1567, 1570, 1572, 1574, 1576, 1579, 1581, 1583, 1586, 1588, 1590, 1592, 1595, 1597, 1599, 1601, 1604, 1606, 1608, 1610, 1612, 1615, 1617, 1619, 1621, 1623, 1626, 1628, 1630, 1632, 1634, 1636, 1639, 1641, 1643, 1645, 1647, 1649, 1651, 1653, 1655, 1658, 1660, 1662, 1664, 1666, 1668, 1670, 1672, 1674, 1676, 1678, 1680, 1682, 1684, 1686, 1688, 1690, 1693, 1695, 1697, 1699, 1701, 1703, 1705, 1707, 1708, 1710, 1712, 1714, 1716, 1718, 1720, 1722, 1724, 1726, 1728, 1730, 1732, 1734, 1736, 1738, 1740, 1742, 1743, 1745, 1747, 1749, 1751, 1753, 1755, 1757, 1759, 1760, 1762, 1764, 1766, 1768, 1770, 1772, 1773, 1775, 1777, 1779, 1781, 1783, 1784, 1786, 1788, 1790, 1792, 1793, 1795, 1797, 1799, 1801, 1802, 1804, 1806, 1808, 1809, 1811, 1813, 1815, 1816, 1818, 1820, 1822, 1823, 1825, 1827, 1829, 1830, 1832, 1834, 1836, 1837, 1839, 1841, 1842, 1844, 1846, 1847, 1849, 1851, 1853, 1854, 1856, 1858, 1859, 1861, 1863, 1864, 1866, 1868, 1869, 1871, 1873, 1874, 1876, 1877, 1879, 1881, 1882, 1884, 1886, 1887, 1889, 1890, 1892, 1894, 1895, 1897, 1898, 1900, 1902, 1903, 1905, 1906, 1908, 1910, 1911, 1913, 1914, 1916, 1917, 1919, 1921, 1922, 1924, 1925, 1927, 1928, 1930, 1931, 1933, 1935, 1936, 1938, 1939, 1941, 1942, 1944, 1945, 1947, 1948, 1950, 1951, 1953, 1954, 1956, 1957, 1959, 1960, 1962, 1963, 1965, 1966, 1968, 1969, 1971, 1972, 1974, 1975, 1977, 1978, 1979, 1981, 1982, 1984, 1985, 1987, 1988, 1990, 1991, 1992, 1994, 1995, 1997, 1998, 2000, 2001, 2002, 2004, 2005, 2007, 2008, 2010, 2011, 2012, 2014, 2015, 2017, 2018, 2019, 2021, 2022, 2024, 2025, 2026, 2028, 2029, 2031, 2032, 2033, 2035, 2036, 2037, 2039, 2040, 2041, 2043, 2044, 2046, 2047, 2048, 2050, 2051, 2052, 2054, 2055, 2056, 2058, 2059, 2060, 2062, 2063, 2064, 2066, 2067, 2068, 2070, 2071, 2072, 2074, 2075, 2076, 2077, 2079, 2080, 2081, 2083, 2084, 2085, 2087, 2088, 2089, 2090, 2092, 2093, 2094, 2096, 2097, 2098, 2099, 2101, 2102, 2103, 2105, 2106, 2107, 2108, 2110, 2111, 2112, 2113, 2115, 2116, 2117, 2118, 2120, 2121, 2122, 2123, 2125, 2126, 2127, 2128, 2130, 2131, 2132, 2133, 2134, 2136, 2137, 2138, 2139, 2141, 2142, 2143, 2144, 2145, 2147, 2148, 2149, 2150, 2151, 2153, 2154, 2155, 2156, 2157, 2159, 2160, 2161, 2162, 2163, 2165, 2166, 2167, 2168, 2169, 2171, 2172, 2173, 2174, 2175, 2176, 2178, 2179, 2180, 2181, 2182, 2183, 2185, 2186, 2187, 2188, 2189, 2190, 2192, 2193, 2194, 2195, 2196, 2197, 2198, 2200, 2201, 2202, 2203, 2204, 2205, 2206, 2207, 2209, 2210, 2211, 2212, 2213, 2214, 2215, 2216, 2218, 2219, 2220, 2221, 2222, 2223, 2224, 2225, 2226, 2228, 2229, 2230, 2231, 2232, 2233, 2234, 2235, 2236, 2237, 2239, 2240, 2241, 2242, 2243, 2244, 2245, 2246, 2247, 2248, 2249, 2250, 2252, 2253, 2254, 2255, 2256, 2257, 2258, 2259, 2260, 2261, 2262, 2263, 2264, 2265, 2266, 2268, 2269, 2270, 2271, 2272, 2273, 2274, 2275, 2276, 2277, 2278, 2279, 2280, 2281, 2282, 2283, 2284, 2285, 2286, 2287, 2288, 2289, 2290, 2292, 2293, 2294, 2295, 2296, 2297, 2298, 2299, 2300, 2301, 2302, 2303, 2304, 2305, 2306, 2307, 2308, 2309, 2310, 2311, 2312, 2313, 2314, 2315, 2316, 2317, 2318, 2319, 2320, 2321, 2322, 2323, 2324, 2325, 2326, 2327, 2328, 2329, 2330, 2331, 2332, 2333, 2334, 2335, 2336, 2337, 2338, 2339, 2340, 2341, 2342, 2342, 2343, 2344, 2345, 2346, 2347, 2348, 2349, 2350, 2351, 2352, 2353, 2354, 2355, 2356, 2357, 2358, 2359, 2360, 2361, 2362, 2363, 2364, 2365, 2366, 2366, 2367, 2368, 2369, 2370, 2371, 2372, 2373, 2374, 2375, 2376, 2377, 2378, 2379, 2380, 2381, 2381, 2382, 2383, 2384, 2385, 2386, 2387, 2388, 2389, 2390, 2391, 2392, 2393, 2393, 2394, 2395, 2396, 2397, 2398, 2399, 2400, 2401, 2402, 2403, 2403, 2404, 2405, 2406, 2407, 2408, 2409, 2410, 2411, 2412, 2413, 2413, 2414, 2415, 2416, 2417, 2418, 2419, 2420, 2421, 2421, 2422, 2423, 2424, 2425, 2426, 2427, 2428, 2429, 2429, 2430, 2431, 2432, 2433, 2434, 2435, 2436, 2436, 2437, 2438, 2439, 2440, 2441, 2442, 2443, 2443, 2444, 2445, 2446, 2447, 2448, 2449, 2449, 2450, 2451, 2452, 2453, 2454, 2455, 2455, 2456, 2457, 2458, 2459, 2460, 2461, 2461, 2462, 2463, 2464, 2465, 2466, 2467, 2467, 2468, 2469, 2470, 2471, 2472, 2472, 2473, 2474, 2475, 2476, 2477, 2477, 2478, 2479, 2480, 2481, 2482, 2482, 2483, 2484, 2485, 2486, 2487, 2487, 2488, 2489, 2490, 2491, 2491, 2492, 2493, 2494, 2495, 2496, 2496, 2497, 2498, 2499, 2500, 2500, 2501, 2502, 2503, 2504, 2505, 2505, 2506, 2507, 2508, 2509, 2509, 2510, 2511, 2512, 2513, 2513, 2514, 2515, 2516, 2517, 2517, 2518, 2519, 2520, 2521, 2521, 2522, 2523, 2524, 2525, 2525, 2526, 2527, 2528, 2529, 2529, 2530, 2531, 2532, 2532, 2533, 2534, 2535, 2536, 2536, 2537, 2538, 2539, 2539, 2540, 2541, 2542, 2543, 2543, 2544, 2545, 2546, 2546, 2547, 2548, 2549, 2550, 2550, 2551, 2552, 2553, 2553, 2554, 2555, 2556, 2556, 2557, 2558, 2559, 2559, 2560, 2561, 2562, 2563, 2563, 2564, 2565, 2566, 2566, 2567, 2568, 2569, 2569, 2570, 2571, 2572, 2572, 2573, 2574, 2575, 2575, 2576, 2577, 2578, 2578, 2579, 2580, 2581, 2581, 2582, 2583, 2584, 2584, 2585, 2586, 2586, 2587, 2588, 2589, 2589, 2590, 2591, 2592, 2592, 2593, 2594, 2595, 2595, 2596, 2597, 2597, 2598, 2599, 2600, 2600, 2601, 2602, 2603, 2603, 2604, 2605, 2605, 2606, 2607, 2608, 2608, 2609, 2610, 2611, 2611, 2612, 2613, 2613, 2614, 2615, 2616, 2616, 2617, 2618, 2618, 2619, 2620, 2621, 2621, 2622, 2623, 2623, 2624, 2625, 2625, 2626, 2627, 2628, 2628, 2629, 2630, 2630, 2631, 2632, 2633, 2633, 2634, 2635, 2635, 2636, 2637, 2637, 2638, 2639, 2640, 2640, 2641, 2642, 2642, 2643, 2644, 2644, 2645, 2646, 2646, 2647, 2648, 2649, 2649, 2650, 2651, 2651, 2652, 2653, 2653, 2654, 2655, 2655, 2656, 2657, 2657, 2658, 2659, 2659, 2660, 2661, 2662, 2662, 2663, 2664, 2664, 2665, 2666, 2666, 2667, 2668, 2668, 2669, 2670, 2670, 2671, 2672, 2672, 2673, 2674, 2674, 2675, 2676, 2676, 2677, 2678, 2678, 2679, 2680, 2680, 2681, 2682, 2682, 2683, 2684, 2684, 2685, 2686, 2686, 2687, 2688, 2688, 2689, 2690, 2690, 2691, 2692, 2692, 2693, 2694, 2694, 2695, 2695, 2696, 2697, 2697, 2698, 2699, 2699, 2700, 2701, 2701, 2702, 2703, 2703, 2704, 2705, 2705, 2706, 2707, 2707, 2708, 2708, 2709, 2710, 2710, 2711, 2712, 2712, 2713, 2714, 2714, 2715, 2716, 2716, 2717, 2717, 2718, 2719, 2719, 2720, 2721, 2721, 2722, 2723, 2723, 2724, 2724, 2725, 2726, 2726, 2727, 2728, 2728, 2729, 2729, 2730, 2731, 2731, 2732, 2733, 2733, 2734, 2734, 2735, 2736, 2736, 2737, 2738, 2738, 2739, 2739, 2740, 2741, 2741, 2742, 2743, 2743, 2744, 2744, 2745, 2746, 2746, 2747, 2748, 2748, 2749, 2749, 2750, 2751, 2751, 2752, 2752, 2753, 2754, 2754, 2755, 2755, 2756, 2757, 2757, 2758, 2759, 2759, 2760, 2760, 2761, 2762, 2762, 2763, 2763, 2764, 2765, 2765, 2766, 2766, 2767, 2768, 2768, 2769, 2769, 2770, 2771, 2771, 2772, 2772, 2773, 2774, 2774, 2775, 2775, 2776, 2777, 2777, 2778, 2778, 2779, 2780, 2780, 2781, 2781, 2782, 2783, 2783, 2784, 2784, 2785, 2786, 2786, 2787, 2787, 2788, 2788, 2789, 2790, 2790, 2791, 2791, 2792, 2793, 2793, 2794, 2794, 2795, 2795, 2796, 2797, 2797, 2798, 2798, 2799, 2800, 2800, 2801, 2801, 2802, 2802, 2803, 2804, 2804, 2805, 2805, 2806, 2807, 2807, 2808, 2808, 2809, 2809, 2810, 2811, 2811, 2812, 2812, 2813, 2813, 2814, 2815, 2815, 2816, 2816, 2817, 2817, 2818, 2819, 2819, 2820, 2820, 2821, 2821, 2822, 2823, 2823, 2824, 2824, 2825, 2825, 2826, 2826, 2827, 2828, 2828, 2829, 2829, 2830, 2830, 2831, 2832, 2832, 2833, 2833, 2834, 2834, 2835, 2835, 2836, 2837, 2837, 2838, 2838, 2839, 2839, 2840, 2840, 2841, 2842, 2842, 2843, 2843, 2844, 2844, 2845, 2845, 2846, 2847, 2847, 2848, 2848, 2849, 2849, 2850, 2850, 2851, 2852, 2852, 2853, 2853, 2854, 2854, 2855, 2855, 2856, 2856, 2857, 2858, 2858, 2859, 2859, 2860, 2860, 2861, 2861, 2862, 2862, 2863, 2863, 2864, 2865, 2865, 2866, 2866, 2867, 2867, 2868, 2868, 2869, 2869, 2870, 2871, 2871, 2872, 2872, 2873, 2873, 2874, 2874, 2875, 2875, 2876, 2876, 2877, 2877, 2878, 2879, 2879, 2880, 2880, 2881, 2881, 2882, 2882, 2883, 2883, 2884, 2884, 2885, 2885, 2886, 2886, 2887, 2888, 2888, 2889, 2889, 2890, 2890, 2891, 2891, 2892, 2892, 2893, 2893, 2894, 2894, 2895, 2895, 2896, 2896, 2897, 2897, 2898, 2899, 2899, 2900, 2900, 2901, 2901, 2902, 2902, 2903, 2903, 2904, 2904, 2905, 2905, 2906, 2906, 2907, 2907, 2908, 2908, 2909, 2909, 2910, 2910, 2911, 2911, 2912, 2912, 2913, 2913, 2914, 2914, 2915, 2916, 2916, 2917, 2917, 2918, 2918, 2919, 2919, 2920, 2920, 2921, 2921, 2922, 2922, 2923, 2923, 2924, 2924, 2925, 2925, 2926, 2926, 2927, 2927, 2928, 2928, 2929, 2929, 2930, 2930, 2931, 2931, 2932, 2932, 2933, 2933, 2934, 2934, 2935, 2935, 2936, 2936, 2937, 2937, 2938, 2938, 2939, 2939, 2940, 2940, 2941, 2941, 2942, 2942, 2943, 2943, 2944, 2944, 2945, 2945, 2946, 2946, 2947, 2947, 2948, 2948, 2949, 2949, 2950, 2950, 2951, 2951, 2952, 2952, 2952, 2953, 2953, 2954, 2954, 2955, 2955, 2956, 2956, 2957, 2957, 2958, 2958, 2959, 2959, 2960, 2960, 2961, 2961, 2962, 2962, 2963, 2963, 2964, 2964, 2965, 2965, 2966, 2966, 2967, 2967, 2968, 2968, 2969, 2969, 2969, 2970, 2970, 2971, 2971, 2972, 2972, 2973, 2973, 2974, 2974, 2975, 2975, 2976, 2976, 2977, 2977, 2978, 2978, 2979, 2979, 2980, 2980, 2980, 2981, 2981, 2982, 2982, 2983, 2983, 2984, 2984, 2985, 2985, 2986, 2986, 2987, 2987, 2988, 2988, 2988, 2989, 2989, 2990, 2990, 2991, 2991, 2992, 2992, 2993, 2993, 2994, 2994, 2995, 2995, 2996, 2996, 2996, 2997, 2997, 2998, 2998, 2999, 2999, 3000, 3000, 3001, 3001, 3002, 3002, 3002, 3003, 3003, 3004, 3004, 3005, 3005, 3006, 3006, 3007, 3007, 3008, 3008, 3008, 3009, 3009, 3010, 3010, 3011, 3011, 3012, 3012, 3013, 3013, 3014, 3014, 3014, 3015, 3015, 3016, 3016, 3017, 3017, 3018, 3018, 3019, 3019, 3019, 3020, 3020, 3021, 3021, 3022, 3022, 3023, 3023, 3024, 3024, 3024, 3025, 3025, 3026, 3026, 3027, 3027, 3028, 3028, 3028, 3029, 3029, 3030, 3030, 3031, 3031, 3032, 3032, 3033, 3033, 3033, 3034, 3034, 3035, 3035, 3036, 3036, 3037, 3037, 3037, 3038, 3038, 3039, 3039, 3040, 3040, 3041, 3041, 3041, 3042, 3042, 3043, 3043, 3044, 3044, 3045, 3045, 3045, 3046, 3046, 3047, 3047, 3048, 3048, 3048, 3049, 3049, 3050, 3050, 3051, 3051, 3052, 3052, 3052, 3053, 3053, 3054, 3054, 3055, 3055, 3055, 3056, 3056, 3057, 3057, 3058, 3058, 3059, 3059, 3059, 3060, 3060, 3061, 3061, 3062, 3062, 3062, 3063, 3063, 3064, 3064, 3065, 3065, 3065, 3066, 3066, 3067, 3067, 3068, 3068, 3068, 3069, 3069, 3070, 3070, 3071, 3071, 3071, 3072, 3072, 3073, 3073, 3074, 3074, 3074, 3075, 3075, 3076, 3076, 3077, 3077, 3077, 3078, 3078, 3079, 3079, 3080, 3080, 3080, 3081, 3081, 3082, 3082, 3083, 3083, 3083, 3084, 3084, 3085, 3085, 3086, 3086, 3086, 3087, 3087, 3088, 3088, 3088, 3089, 3089, 3090, 3090, 3091, 3091, 3091, 3092, 3092, 3093, 3093, 3093, 3094, 3094, 3095, 3095, 3096, 3096, 3096, 3097, 3097, 3098, 3098, 3098, 3099, 3099, 3100, 3100, 3101, 3101, 3101, 3102, 3102, 3103, 3103, 3103, 3104, 3104, 3105, 3105, 3106, 3106, 3106, 3107, 3107, 3108, 3108, 3108, 3109, 3109, 3110, 3110, 3110, 3111, 3111, 3112, 3112, 3113, 3113, 3113, 3114, 3114, 3115, 3115, 3115, 3116, 3116, 3117, 3117, 3117, 3118, 3118, 3119, 3119, 3119, 3120, 3120, 3121, 3121, 3121, 3122, 3122, 3123, 3123, 3123, 3124, 3124, 3125, 3125, 3126, 3126, 3126, 3127, 3127, 3128, 3128, 3128, 3129, 3129, 3130, 3130, 3130, 3131, 3131, 3132, 3132, 3132, 3133, 3133, 3134, 3134, 3134, 3135, 3135, 3136, 3136, 3136, 3137, 3137, 3138, 3138, 3138, 3139, 3139, 3140, 3140, 3140, 3141, 3141, 3141, 3142, 3142, 3143, 3143, 3143, 3144, 3144, 3145, 3145, 3145, 3146, 3146, 3147, 3147, 3147, 3148, 3148, 3149, 3149, 3149, 3150, 3150, 3151, 3151, 3151, 3152, 3152, 3153, 3153, 3153, 3154, 3154, 3154, 3155, 3155, 3156, 3156, 3156, 3157, 3157, 3158, 3158, 3158, 3159, 3159, 3160, 3160, 3160, 3161, 3161, 3161, 3162, 3162, 3163, 3163, 3163, 3164, 3164, 3165, 3165, 3165, 3166, 3166, 3166, 3167, 3167, 3168, 3168, 3168, 3169, 3169, 3170, 3170, 3170, 3171, 3171, 3171, 3172, 3172, 3173, 3173, 3173, 3174, 3174, 3175, 3175, 3175, 3176, 3176, 3176, 3177, 3177, 3178, 3178, 3178, 3179, 3179, 3179, 3180, 3180, 3181, 3181, 3181, 3182, 3182, 3183, 3183, 3183, 3184, 3184, 3184, 3185, 3185, 3186, 3186, 3186, 3187, 3187, 3187, 3188, 3188, 3189, 3189, 3189, 3190, 3190, 3190, 3191, 3191, 3192, 3192, 3192, 3193, 3193, 3193, 3194, 3194, 3195, 3195, 3195, 3196, 3196, 3196, 3197, 3197, 3198, 3198, 3198, 3199, 3199, 3199, 3200, 3200, 3200, 3201, 3201, 3202, 3202, 3202, 3203, 3203, 3203, 3204, 3204, 3205, 3205, 3205, 3206, 3206, 3206, 3207, 3207, 3207, 3208, 3208, 3209, 3209, 3209, 3210, 3210, 3210, 3211, 3211, 3212, 3212, 3212, 3213, 3213, 3213, 3214, 3214, 3214, 3215, 3215, 3216, 3216, 3216, 3217, 3217, 3217, 3218, 3218, 3218, 3219, 3219, 3220, 3220, 3220, 3221, 3221, 3221, 3222, 3222, 3222, 3223, 3223, 3224, 3224, 3224, 3225, 3225, 3225, 3226, 3226, 3226, 3227, 3227, 3227, 3228, 3228, 3229, 3229, 3229, 3230, 3230, 3230, 3231, 3231, 3231, 3232, 3232, 3233, 3233, 3233, 3234, 3234, 3234, 3235, 3235, 3235, 3236, 3236, 3236, 3237, 3237, 3237, 3238, 3238, 3239, 3239, 3239, 3240, 3240, 3240, 3241, 3241, 3241, 3242, 3242, 3242, 3243, 3243, 3244, 3244, 3244, 3245, 3245, 3245, 3246, 3246, 3246, 3247, 3247, 3247, 3248, 3248, 3248, 3249, 3249, 3249, 3250, 3250, 3251, 3251, 3251, 3252, 3252, 3252, 3253, 3253, 3253, 3254, 3254, 3254, 3255, 3255, 3255, 3256, 3256, 3256, 3257, 3257, 3258, 3258, 3258, 3259, 3259, 3259, 3260, 3260, 3260, 3261, 3261, 3261, 3262, 3262, 3262, 3263, 3263, 3263, 3264, 3264, 3264, 3265, 3265, 3265, 3266, 3266, 3267, 3267, 3267, 3268, 3268, 3268, 3269, 3269, 3269, 3270, 3270, 3270, 3271, 3271, 3271, 3272, 3272, 3272, 3273, 3273, 3273, 3274, 3274, 3274, 3275, 3275, 3275, 3276, 3276, 3276, 3277, 3277, 3277, 3278, 3278, 3278, 3279, 3279, 3280, 3280, 3280, 3281, 3281, 3281, 3282, 3282, 3282, 3283, 3283, 3283, 3284, 3284, 3284, 3285, 3285, 3285, 3286, 3286, 3286, 3287, 3287, 3287, 3288, 3288, 3288, 3289, 3289, 3289, 3290, 3290, 3290, 3291, 3291, 3291, 3292, 3292, 3292, 3293, 3293, 3293, 3294, 3294, 3294, 3295, 3295, 3295, 3296, 3296, 3296, 3297, 3297, 3297, 3298, 3298, 3298, 3299, 3299, 3299, 3300, 3300, 3300, 3301, 3301, 3301, 3302, 3302, 3302, 3303, 3303, 3303, 3304, 3304, 3304, 3305, 3305, 3305, 3306, 3306, 3306, 3307, 3307, 3307, 3308, 3308, 3308, 3309, 3309, 3309, 3310, 3310, 3310, 3310, 3311, 3311, 3311, 3312, 3312, 3312, 3313, 3313, 3313, 3314, 3314, 3314, 3315, 3315, 3315, 3316, 3316, 3316, 3317, 3317, 3317, 3318, 3318, 3318, 3319, 3319, 3319, 3320, 3320, 3320, 3321, 3321, 3321, 3322, 3322, 3322, 3323, 3323, 3323, 3324, 3324, 3324, 3324, 3325, 3325, 3325, 3326, 3326, 3326, 3327, 3327, 3327, 3328, 3328, 3328, 3329, 3329, 3329, 3330, 3330, 3330, 3331, 3331, 3331, 3332, 3332, 3332, 3332, 3333, 3333, 3333, 3334, 3334, 3334, 3335, 3335, 3335, 3336, 3336, 3336, 3337, 3337, 3337, 3338, 3338, 3338, 3339, 3339, 3339, 3340, 3340, 3340, 3340, 3341, 3341, 3341, 3342, 3342, 3342, 3343, 3343, 3343, 3344, 3344, 3344, 3345, 3345, 3345, 3346, 3346, 3346, 3346, 3347, 3347, 3347, 3348, 3348, 3348, 3349, 3349, 3349, 3350, 3350, 3350, 3351, 3351, 3351, 3351, 3352, 3352, 3352, 3353, 3353, 3353, 3354, 3354, 3354, 3355, 3355, 3355, 3356, 3356, 3356, 3356, 3357, 3357, 3357, 3358, 3358, 3358, 3359, 3359, 3359, 3360, 3360, 3360, 3360, 3361, 3361, 3361, 3362, 3362, 3362, 3363, 3363, 3363, 3364, 3364, 3364, 3365, 3365, 3365, 3365, 3366, 3366, 3366, 3367, 3367, 3367, 3368, 3368, 3368, 3369, 3369, 3369, 3369, 3370, 3370, 3370, 3371, 3371, 3371, 3372, 3372, 3372, 3372, 3373, 3373, 3373, 3374, 3374, 3374, 3375, 3375, 3375, 3376, 3376, 3376, 3376, 3377, 3377, 3377, 3378, 3378, 3378, 3379, 3379, 3379, 3379, 3380, 3380, 3380, 3381, 3381, 3381, 3382, 3382, 3382, 3383, 3383, 3383, 3383, 3384, 3384, 3384, 3385, 3385, 3385, 3386, 3386, 3386, 3386, 3387, 3387, 3387, 3388, 3388, 3388, 3389, 3389, 3389, 3389, 3390, 3390, 3390, 3391, 3391, 3391, 3392, 3392, 3392, 3392, 3393, 3393, 3393, 3394, 3394, 3394, 3395, 3395, 3395, 3395, 3396, 3396, 3396, 3397, 3397, 3397, 3397, 3398, 3398, 3398, 3399, 3399, 3399, 3400, 3400, 3400, 3400, 3401, 3401, 3401, 3402, 3402, 3402, 3403, 3403, 3403, 3403, 3404, 3404, 3404, 3405, 3405, 3405, 3405, 3406, 3406, 3406, 3407, 3407, 3407, 3408, 3408, 3408, 3408, 3409, 3409, 3409, 3410, 3410, 3410, 3410, 3411, 3411, 3411, 3412, 3412, 3412, 3413, 3413, 3413, 3413, 3414, 3414, 3414, 3415, 3415, 3415, 3415, 3416, 3416, 3416, 3417, 3417, 3417, 3417, 3418, 3418, 3418, 3419, 3419, 3419, 3420, 3420, 3420, 3420, 3421, 3421, 3421, 3422, 3422, 3422, 3422, 3423, 3423, 3423, 3424, 3424, 3424, 3424, 3425, 3425, 3425, 3426, 3426, 3426, 3426, 3427, 3427, 3427, 3428, 3428, 3428, 3428, 3429, 3429, 3429, 3430, 3430, 3430, 3430, 3431, 3431, 3431, 3432, 3432, 3432, 3432, 3433, 3433, 3433, 3434, 3434, 3434, 3434, 3435, 3435, 3435, 3436, 3436, 3436, 3436, 3437, 3437, 3437, 3438, 3438, 3438, 3438, 3439, 3439, 3439, 3440, 3440, 3440, 3440, 3441, 3441, 3441, 3442, 3442, 3442, 3442, 3443, 3443, 3443, 3444, 3444, 3444, 3444, 3445, 3445, 3445, 3445, 3446, 3446, 3446, 3447, 3447, 3447, 3447, 3448, 3448, 3448, 3449, 3449, 3449, 3449, 3450, 3450, 3450, 3451, 3451, 3451, 3451, 3452, 3452, 3452, 3452, 3453, 3453, 3453, 3454, 3454, 3454, 3454, 3455, 3455, 3455, 3456, 3456, 3456, 3456, 3457, 3457, 3457, 3458, 3458, 3458, 3458, 3459, 3459, 3459, 3459, 3460, 3460, 3460, 3461, 3461, 3461, 3461, 3462, 3462, 3462, 3462, 3463, 3463, 3463, 3464, 3464, 3464, 3464, 3465, 3465, 3465, 3466, 3466, 3466, 3466, 3467, 3467, 3467, 3467, 3468, 3468, 3468, 3469, 3469, 3469, 3469, 3470, 3470, 3470, 3470, 3471, 3471, 3471, 3472, 3472, 3472, 3472, 3473, 3473, 3473, 3473, 3474, 3474, 3474, 3475, 3475, 3475, 3475, 3476, 3476, 3476, 3476, 3477, 3477, 3477, 3478, 3478, 3478, 3478, 3479, 3479, 3479, 3479, 3480, 3480, 3480, 3481, 3481, 3481, 3481, 3482, 3482, 3482, 3482, 3483, 3483, 3483, 3484, 3484, 3484, 3484, 3485, 3485, 3485, 3485, 3486, 3486, 3486, 3486, 3487, 3487, 3487, 3488, 3488, 3488, 3488, 3489, 3489, 3489, 3489, 3490, 3490, 3490, 3490, 3491, 3491, 3491, 3492, 3492, 3492, 3492, 3493, 3493, 3493, 3493, 3494, 3494, 3494, 3495, 3495, 3495, 3495, 3496, 3496, 3496, 3496, 3497, 3497, 3497, 3497, 3498, 3498, 3498, 3498, 3499, 3499, 3499, 3500, 3500, 3500, 3500, 3501, 3501, 3501, 3501, 3502, 3502, 3502, 3502, 3503, 3503, 3503, 3504, 3504, 3504, 3504, 3505, 3505, 3505, 3505, 3506, 3506, 3506, 3506, 3507, 3507, 3507, 3507, 3508, 3508, 3508, 3509, 3509, 3509, 3509, 3510, 3510, 3510, 3510, 3511, 3511, 3511, 3511, 3512, 3512, 3512, 3512, 3513, 3513, 3513, 3513, 3514, 3514};

static inline struct sde_kms *_sde_connector_get_kms(struct drm_connector *conn)
{
	struct msm_drm_private *priv;

	if (!conn || !conn->dev || !conn->dev->dev_private) {
		SDE_ERROR("invalid connector\n");
		return NULL;
	}
	priv = conn->dev->dev_private;
	if (!priv || !priv->kms) {
		SDE_ERROR("invalid kms\n");
		return NULL;
	}

	return to_sde_kms(priv->kms);
}

static int sde_backlight_device_update_status(struct backlight_device *bd)
{
	int brightness;
	struct dsi_display *display;
	struct sde_connector *c_conn = bl_get_data(bd);
	int bl_lvl;
	struct drm_event event;
	int rc = 0;
	struct sde_kms *sde_kms;
	struct sde_vm_ops *vm_ops;

	sde_kms = _sde_connector_get_kms(&c_conn->base);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return -EINVAL;
	}

	brightness = bd->props.brightness;

	if ((bd->props.power != FB_BLANK_UNBLANK) ||
			(bd->props.state & BL_CORE_FBBLANK) ||
			(bd->props.state & BL_CORE_SUSPENDED))
		brightness = 0;

	display = (struct dsi_display *) c_conn->display;
	if (brightness > display->panel->bl_config.bl_max_level)
		brightness = display->panel->bl_config.bl_max_level;
	if (brightness > c_conn->thermal_max_brightness)
		brightness = c_conn->thermal_max_brightness;

	if (brightness && brightness < display->panel->bl_config.bl_min_level)
		brightness = display->panel->bl_config.bl_min_level;

	/* map UI brightness into driver backlight level with rounding */
	bl_lvl = mult_frac(brightness, display->panel->bl_config.bl_max_level,
			display->panel->bl_config.brightness_max_level);

	if (!bl_lvl && brightness)
		bl_lvl = 1;

	pr_info("%s: map bl_lvl from %d to %d", __func__, bl_lvl, bl_lut[bl_lvl]);
	bl_lvl = bl_lut[bl_lvl];

	if (!c_conn->allow_bl_update) {
		c_conn->unset_bl_level = bl_lvl;
		return 0;
	}

	sde_vm_lock(sde_kms);

	vm_ops = sde_vm_get_ops(sde_kms);
	if (vm_ops && vm_ops->vm_owns_hw && !vm_ops->vm_owns_hw(sde_kms)) {
		sde_vm_unlock(sde_kms);
		SDE_DEBUG("skipping bl update due to HW unavailablity\n");
		goto done;
	}

	sde_vm_unlock(sde_kms);

	if (c_conn->ops.set_backlight) {
		/* skip notifying user space if bl is 0 */
		if (brightness != 0) {
			event.type = DRM_EVENT_SYS_BACKLIGHT;
			event.length = sizeof(u32);
			msm_mode_object_event_notify(&c_conn->base.base,
				c_conn->base.dev, &event, (u8 *)&brightness);
		}
		rc = c_conn->ops.set_backlight(&c_conn->base,
				c_conn->display, bl_lvl);
		c_conn->unset_bl_level = 0;
	}

done:

	return rc;
}

static int sde_backlight_device_get_brightness(struct backlight_device *bd)
{
	return 0;
}

static const struct backlight_ops sde_backlight_device_ops = {
	.update_status = sde_backlight_device_update_status,
	.get_brightness = sde_backlight_device_get_brightness,
};

static int sde_backlight_cooling_cb(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct sde_connector *c_conn;
	struct backlight_device *bd = (struct backlight_device *)data;

	c_conn = bl_get_data(bd);
	SDE_DEBUG("bl: thermal max brightness cap:%lu\n", val);
	c_conn->thermal_max_brightness = val;

	sde_backlight_device_update_status(bd);
	return 0;
}

static int sde_backlight_setup(struct sde_connector *c_conn,
					struct drm_device *dev)
{
	struct backlight_properties props;
	struct dsi_display *display;
	struct dsi_backlight_config *bl_config;
	struct sde_kms *sde_kms;
	static int display_count;
	char bl_node_name[BL_NODE_NAME_SIZE];

	sde_kms = _sde_connector_get_kms(&c_conn->base);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return -EINVAL;
	} else if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		return 0;
	}

	display = (struct dsi_display *) c_conn->display;
	bl_config = &display->panel->bl_config;

	if (bl_config->type != DSI_BACKLIGHT_DCS &&
		sde_in_trusted_vm(sde_kms))
		return 0;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.power = FB_BLANK_UNBLANK;
	props.max_brightness = bl_config->brightness_max_level;
	props.brightness = bl_config->brightness_default_level;
	snprintf(bl_node_name, BL_NODE_NAME_SIZE, "panel%u-backlight",
							display_count);
	c_conn->bl_device = backlight_device_register(bl_node_name, dev->dev,
			c_conn, &sde_backlight_device_ops, &props);
	if (IS_ERR_OR_NULL(c_conn->bl_device)) {
		SDE_ERROR("Failed to register backlight: %ld\n",
				    PTR_ERR(c_conn->bl_device));
		c_conn->bl_device = NULL;
		return -ENODEV;
	}
	c_conn->thermal_max_brightness = bl_config->brightness_max_level;

	/**
	 * In TVM, thermal cooling device is not enabled. Registering with dummy
	 * thermal device will return a NULL leading to a failure. So skip it.
	 */
	if (sde_in_trusted_vm(sde_kms))
		goto done;

	c_conn->n.notifier_call = sde_backlight_cooling_cb;
	c_conn->cdev = backlight_cdev_register(dev->dev, c_conn->bl_device,
							&c_conn->n);
	if (IS_ERR_OR_NULL(c_conn->cdev)) {
		SDE_ERROR("Failed to register backlight cdev: %ld\n",
				    PTR_ERR(c_conn->cdev));
		backlight_device_unregister(c_conn->bl_device);
		c_conn->bl_device = NULL;
		return -ENODEV;
	}

done:
	display_count++;

	return 0;
}

int sde_connector_trigger_event(void *drm_connector,
		uint32_t event_idx, uint32_t instance_idx,
		uint32_t data0, uint32_t data1,
		uint32_t data2, uint32_t data3)
{
	struct sde_connector *c_conn;
	unsigned long irq_flags;
	int (*cb_func)(uint32_t event_idx,
			uint32_t instance_idx, void *usr,
			uint32_t data0, uint32_t data1,
			uint32_t data2, uint32_t data3);
	void *usr;
	int rc = 0;

	/*
	 * This function may potentially be called from an ISR context, so
	 * avoid excessive logging/etc.
	 */
	if (!drm_connector)
		return -EINVAL;
	else if (event_idx >= SDE_CONN_EVENT_COUNT)
		return -EINVAL;
	c_conn = to_sde_connector(drm_connector);

	spin_lock_irqsave(&c_conn->event_lock, irq_flags);
	cb_func = c_conn->event_table[event_idx].cb_func;
	usr = c_conn->event_table[event_idx].usr;
	spin_unlock_irqrestore(&c_conn->event_lock, irq_flags);

	if (cb_func)
		rc = cb_func(event_idx, instance_idx, usr,
			data0, data1, data2, data3);
	else
		rc = -EAGAIN;

	return rc;
}

int sde_connector_register_event(struct drm_connector *connector,
		uint32_t event_idx,
		int (*cb_func)(uint32_t event_idx,
			uint32_t instance_idx, void *usr,
			uint32_t data0, uint32_t data1,
			uint32_t data2, uint32_t data3),
		void *usr)
{
	struct sde_connector *c_conn;
	unsigned long irq_flags;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return -EINVAL;
	} else if (event_idx >= SDE_CONN_EVENT_COUNT) {
		SDE_ERROR("conn%d, invalid event %d\n",
				connector->base.id, event_idx);
		return -EINVAL;
	}
	c_conn = to_sde_connector(connector);

	spin_lock_irqsave(&c_conn->event_lock, irq_flags);
	c_conn->event_table[event_idx].cb_func = cb_func;
	c_conn->event_table[event_idx].usr = usr;
	spin_unlock_irqrestore(&c_conn->event_lock, irq_flags);

	/* optionally notify display of event registration */
	if (c_conn->ops.enable_event && c_conn->display)
		c_conn->ops.enable_event(connector, event_idx,
				cb_func != NULL, c_conn->display);
	return 0;
}

void sde_connector_unregister_event(struct drm_connector *connector,
		uint32_t event_idx)
{
	(void)sde_connector_register_event(connector, event_idx, 0, 0);
}

static void _sde_connector_install_dither_property(struct drm_device *dev,
		struct sde_kms *sde_kms, struct sde_connector *c_conn)
{
	char prop_name[DRM_PROP_NAME_LEN];
	struct sde_mdss_cfg *catalog = NULL;
	u32 version = 0;

	if (!dev || !sde_kms || !c_conn) {
		SDE_ERROR("invld args (s), dev %pK, sde_kms %pK, c_conn %pK\n",
				dev, sde_kms, c_conn);
		return;
	}

	catalog = sde_kms->catalog;
	version = SDE_COLOR_PROCESS_MAJOR(
			catalog->pingpong[0].sblk->dither.version);
	snprintf(prop_name, ARRAY_SIZE(prop_name), "%s%d",
			"SDE_PP_DITHER_V", version);
	switch (version) {
	case 1:
	case 2:
		msm_property_install_blob(&c_conn->property_info, prop_name,
			DRM_MODE_PROP_BLOB,
			CONNECTOR_PROP_PP_DITHER);
		break;
	default:
		SDE_ERROR("unsupported dither version %d\n", version);
		return;
	}
}

int sde_connector_get_dither_cfg(struct drm_connector *conn,
			struct drm_connector_state *state, void **cfg,
			size_t *len, bool idle_pc)
{
	struct sde_connector *c_conn = NULL;
	struct sde_connector_state *c_state = NULL;
	size_t dither_sz = 0;
	bool is_dirty;
	u32 *p = (u32 *)cfg;

	if (!conn || !state || !p) {
		SDE_ERROR("invalid arguments\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(conn);
	c_state = to_sde_connector_state(state);

	is_dirty = msm_property_is_dirty(&c_conn->property_info,
			&c_state->property_state,
			CONNECTOR_PROP_PP_DITHER);

	if (!is_dirty && !idle_pc) {
		return -ENODATA;
	} else if (is_dirty || idle_pc) {
		*cfg = msm_property_get_blob(&c_conn->property_info,
				&c_state->property_state,
				&dither_sz,
				CONNECTOR_PROP_PP_DITHER);
		/*
		 * in idle_pc use case return early,
		 * when dither is already disabled.
		 */
		if (idle_pc && *cfg == NULL)
			return -ENODATA;
		/* disable dither based on user config data */
		else if (*cfg == NULL)
			return 0;
	}
	*len = dither_sz;
	return 0;
}

static void sde_connector_get_avail_res_info(struct drm_connector *conn,
		struct msm_resource_caps_info *avail_res)
{
	struct sde_kms *sde_kms;
	struct drm_encoder *drm_enc = NULL;

	sde_kms = _sde_connector_get_kms(conn);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	if (conn->state && conn->state->best_encoder)
		drm_enc = conn->state->best_encoder;
	else
		drm_enc = conn->encoder;

	sde_rm_get_resource_info(&sde_kms->rm, drm_enc, avail_res);

	avail_res->max_mixer_width = sde_kms->catalog->max_mixer_width;
}

int sde_connector_get_lm_cnt_from_topology(struct drm_connector *conn,
		const struct drm_display_mode *drm_mode)
{
	struct sde_connector *c_conn;

	c_conn = to_sde_connector(conn);

	if (!c_conn || c_conn->connector_type != DRM_MODE_CONNECTOR_DSI ||
		!c_conn->ops.get_num_lm_from_mode)
		return -EINVAL;

	return c_conn->ops.get_num_lm_from_mode(c_conn->display, drm_mode);
}

int sde_connector_get_mode_info(struct drm_connector *conn,
		const struct drm_display_mode *drm_mode,
		struct msm_mode_info *mode_info)
{
	struct sde_connector *sde_conn;
	struct msm_resource_caps_info avail_res;

	memset(&avail_res, 0, sizeof(avail_res));

	sde_conn = to_sde_connector(conn);

	if (!sde_conn)
		return -EINVAL;

	sde_connector_get_avail_res_info(conn, &avail_res);

	return sde_conn->ops.get_mode_info(conn, drm_mode,
			mode_info, sde_conn->display, &avail_res);
}

int sde_connector_state_get_mode_info(struct drm_connector_state *conn_state,
	struct msm_mode_info *mode_info)
{
	struct sde_connector_state *sde_conn_state = NULL;

	if (!conn_state || !mode_info) {
		SDE_ERROR("Invalid arguments\n");
		return -EINVAL;
	}

	sde_conn_state = to_sde_connector_state(conn_state);
	memcpy(mode_info, &sde_conn_state->mode_info,
		sizeof(sde_conn_state->mode_info));

	return 0;
}

static int sde_connector_handle_panel_id(uint32_t event_idx,
			uint32_t instance_idx, void *usr,
			uint32_t data0, uint32_t data1,
			uint32_t data2, uint32_t data3)
{
	struct sde_connector *c_conn = usr;
	int i;
	u64 panel_id;
	u8 arr[8], shift;
	u64 mask = 0xff;

	if (!c_conn)
		return -EINVAL;

	panel_id = (((u64)data0) << 31) | data1;
	if (panel_id == ~0x0)
		return 0;

	for (i = 0; i < 8; i++) {
		shift = 8 * i;
		arr[7 - i] = (u8)((panel_id & (mask << shift)) >> shift);
	}
	/* update the panel id */
	msm_property_set_blob(&c_conn->property_info,
		  &c_conn->blob_panel_id, arr, sizeof(arr),
		  CONNECTOR_PROP_DEMURA_PANEL_ID);
	sde_connector_register_event(&c_conn->base,
			SDE_CONN_EVENT_PANEL_ID, NULL, c_conn);
	return 0;
}

static int sde_connector_handle_disp_recovery(uint32_t event_idx,
			uint32_t instance_idx, void *usr,
			uint32_t data0, uint32_t data1,
			uint32_t data2, uint32_t data3)
{
	struct sde_connector *c_conn = usr;
	int rc = 0;

	if (!c_conn)
		return -EINVAL;

	rc = sde_kms_handle_recovery(c_conn->encoder);

	return rc;
}

int sde_connector_get_info(struct drm_connector *connector,
		struct msm_display_info *info)
{
	struct sde_connector *c_conn;

	if (!connector || !info) {
		SDE_ERROR("invalid argument(s), conn %pK, info %pK\n",
				connector, info);
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);

	if (!c_conn->display || !c_conn->ops.get_info) {
		SDE_ERROR("display info not supported for %pK\n",
				c_conn->display);
		return -EINVAL;
	}

	return c_conn->ops.get_info(&c_conn->base, info, c_conn->display);
}

void sde_connector_schedule_status_work(struct drm_connector *connector,
		bool en)
{
	struct sde_connector *c_conn;
	struct msm_display_info info;

	c_conn = to_sde_connector(connector);
	if (!c_conn)
		return;

	/* Return if there is no change in ESD status check condition */
	if (en == c_conn->esd_status_check)
		return;

	sde_connector_get_info(connector, &info);
	if ((c_conn->ops.force_esd_disable &&
		(c_conn->ops.force_esd_disable(c_conn->display) == false)) &&
		(c_conn->ops.check_status &&
		(info.capabilities & MSM_DISPLAY_ESD_ENABLED))) {
		if (en) {
			u32 interval;

			/*
			 * If debugfs property is not set then take
			 * default value
			 */
			interval = c_conn->esd_status_interval ?
				c_conn->esd_status_interval :
					STATUS_CHECK_INTERVAL_MS;
			/* Schedule ESD status check */
			schedule_delayed_work(&c_conn->status_work,
				msecs_to_jiffies(interval));
			c_conn->esd_status_check = true;
		} else {
			/* Cancel any pending ESD status check */
			cancel_delayed_work_sync(&c_conn->status_work);
			c_conn->esd_status_check = false;
		}
	}
}

static int _sde_connector_update_power_locked(struct sde_connector *c_conn)
{
	struct drm_connector *connector;
	void *display;
	int (*set_power)(struct drm_connector *conn, int status, void *disp);
	int mode, rc = 0;

	if (!c_conn)
		return -EINVAL;
	connector = &c_conn->base;

	switch (c_conn->dpms_mode) {
	case DRM_MODE_DPMS_ON:
		mode = c_conn->lp_mode;
		break;
	case DRM_MODE_DPMS_STANDBY:
		mode = SDE_MODE_DPMS_STANDBY;
		break;
	case DRM_MODE_DPMS_SUSPEND:
		mode = SDE_MODE_DPMS_SUSPEND;
		break;
	case DRM_MODE_DPMS_OFF:
		mode = SDE_MODE_DPMS_OFF;
		break;
	default:
		mode = c_conn->lp_mode;
		SDE_ERROR("conn %d dpms set to unrecognized mode %d\n",
				connector->base.id, mode);
		break;
	}

	SDE_EVT32(connector->base.id, c_conn->dpms_mode, c_conn->lp_mode, mode);
	SDE_DEBUG("conn %d - dpms %d, lp %d, panel %d\n", connector->base.id,
			c_conn->dpms_mode, c_conn->lp_mode, mode);

	if (mode != c_conn->last_panel_power_mode && c_conn->ops.set_power) {
		display = c_conn->display;
		set_power = c_conn->ops.set_power;

		mutex_unlock(&c_conn->lock);
		rc = set_power(connector, mode, display);
		mutex_lock(&c_conn->lock);
	}
	c_conn->last_panel_power_mode = mode;

	mutex_unlock(&c_conn->lock);
	if (mode != SDE_MODE_DPMS_ON)
		sde_connector_schedule_status_work(connector, false);
	else
		sde_connector_schedule_status_work(connector, true);
	mutex_lock(&c_conn->lock);

	return rc;
}

static int _sde_connector_update_bl_scale(struct sde_connector *c_conn)
{
	struct dsi_display *dsi_display;
	struct dsi_backlight_config *bl_config;
	int rc = 0;

	if (!c_conn) {
		SDE_ERROR("Invalid params sde_connector null\n");
		return -EINVAL;
	}

	dsi_display = c_conn->display;
	if (!dsi_display || !dsi_display->panel) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
			dsi_display,
			((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	bl_config = &dsi_display->panel->bl_config;

	if (!c_conn->allow_bl_update) {
		c_conn->unset_bl_level = bl_config->bl_level;
		return 0;
	}

	if (c_conn->unset_bl_level)
		bl_config->bl_level = c_conn->unset_bl_level;

	bl_config->bl_scale = c_conn->bl_scale > MAX_BL_SCALE_LEVEL ?
			MAX_BL_SCALE_LEVEL : c_conn->bl_scale;
	bl_config->bl_scale_sv = c_conn->bl_scale_sv > MAX_SV_BL_SCALE_LEVEL ?
			MAX_SV_BL_SCALE_LEVEL : c_conn->bl_scale_sv;

	SDE_DEBUG("bl_scale = %u, bl_scale_sv = %u, bl_level = %u\n",
		bl_config->bl_scale, bl_config->bl_scale_sv,
		bl_config->bl_level);
	rc = c_conn->ops.set_backlight(&c_conn->base,
			dsi_display, bl_config->bl_level);
	c_conn->unset_bl_level = 0;

	return rc;
}

void sde_connector_set_colorspace(struct sde_connector *c_conn)
{
	int rc = 0;

	if (c_conn->ops.set_colorspace)
		rc = c_conn->ops.set_colorspace(&c_conn->base,
			c_conn->display);

	if (rc)
		SDE_ERROR_CONN(c_conn, "cannot apply new colorspace %d\n", rc);

}

void sde_connector_set_qsync_params(struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;
	u32 qsync_propval = 0;
	bool prop_dirty;

	if (!connector)
		return;

	c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(connector->state);
	c_conn->qsync_updated = false;

	prop_dirty = msm_property_is_dirty(&c_conn->property_info,
					&c_state->property_state,
					CONNECTOR_PROP_QSYNC_MODE);
	if (prop_dirty) {
		qsync_propval = sde_connector_get_property(c_conn->base.state,
						CONNECTOR_PROP_QSYNC_MODE);
		if (qsync_propval != c_conn->qsync_mode) {
			SDE_DEBUG("updated qsync mode %d -> %d\n",
					c_conn->qsync_mode, qsync_propval);
			c_conn->qsync_updated = true;
			c_conn->qsync_mode = qsync_propval;
		}
	}
}

void sde_connector_complete_qsync_commit(struct drm_connector *conn,
				struct msm_display_conn_params *params)
{
	struct sde_connector *c_conn;

	if (!conn || !params) {
		SDE_ERROR("invalid params\n");
		return;
	}

	c_conn = to_sde_connector(conn);

	if (c_conn && c_conn->qsync_updated &&
		(c_conn->qsync_mode == SDE_RM_QSYNC_ONE_SHOT_MODE)) {
		/* Reset qsync states if mode is one shot */
		params->qsync_mode = c_conn->qsync_mode = 0;
		params->qsync_update = true;
		SDE_EVT32(conn->base.id, c_conn->qsync_mode);
	}
}

static int _sde_connector_update_hdr_metadata(struct sde_connector *c_conn,
		struct sde_connector_state *c_state)
{
	int rc = 0;

	if (c_conn->ops.config_hdr)
		rc = c_conn->ops.config_hdr(&c_conn->base, c_conn->display,
				c_state);

	if (rc)
		SDE_ERROR_CONN(c_conn, "cannot apply hdr metadata %d\n", rc);

	SDE_DEBUG_CONN(c_conn, "updated hdr metadata: %d\n", rc);
	return rc;
}

static int _sde_connector_update_param(struct sde_connector *c_conn,
			struct msm_param_info *param_info)
{
	struct dsi_display *dsi_display;
	int rc = 0;

	if (!c_conn) {
		SDE_ERROR("Invalid params sde_connector null\n");
		return -EINVAL;
	}

	dsi_display = c_conn->display;
	if (dsi_display && c_conn->ops.set_param)
		rc = c_conn->ops.set_param(dsi_display, param_info);

	return rc;
}

static int _sde_connector_update_dirty_properties(
				struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;
	int idx;

	if (!connector) {
		SDE_ERROR("invalid argument\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(connector->state);

	mutex_lock(&c_conn->property_info.property_lock);
	while ((idx = msm_property_pop_dirty(&c_conn->property_info,
					&c_state->property_state)) >= 0) {
		switch (idx) {
		case CONNECTOR_PROP_LP:
			mutex_lock(&c_conn->lock);
			c_conn->lp_mode = sde_connector_get_property(
					connector->state, CONNECTOR_PROP_LP);
			_sde_connector_update_power_locked(c_conn);
			mutex_unlock(&c_conn->lock);
			break;
		case CONNECTOR_PROP_HDR_METADATA:
			_sde_connector_update_hdr_metadata(c_conn, c_state);
			break;
		default:
			/* nothing to do for most properties */
			break;
		}
	}
	mutex_unlock(&c_conn->property_info.property_lock);

	/* if colorspace needs to be updated do it first */
	if (c_conn->colorspace_updated) {
		c_conn->colorspace_updated = false;
		sde_connector_set_colorspace(c_conn);
	}

	/*
	 * Special handling for postproc properties and
	 * for updating backlight if any unset backlight level is present
	 */
	if (c_conn->bl_scale_dirty || c_conn->unset_bl_level) {
		_sde_connector_update_bl_scale(c_conn);
		c_conn->bl_scale_dirty = false;
	}

	return 0;
}

struct sde_connector_dyn_hdr_metadata *sde_connector_get_dyn_hdr_meta(
		struct drm_connector *connector)
{
	struct sde_connector_state *c_state;

	if (!connector)
		return NULL;

	c_state = to_sde_connector_state(connector->state);
	return &c_state->dyn_hdr_meta;
}

static bool sde_connector_is_fod_enabled(struct sde_connector *c_conn)
{
	struct drm_connector *connector = &c_conn->base;

	if (!connector->state || !connector->state->crtc)
		return false;

	return sde_crtc_is_fod_enabled(connector->state->crtc->state);
}

struct dsi_panel *sde_connector_panel(struct sde_connector *c_conn)
{
	struct dsi_display *display = (struct dsi_display *)c_conn->display;

	return display ? display->panel : NULL;
}

static void sde_connector_pre_update_fod_hbm(struct sde_connector *c_conn)
{
	struct dsi_panel *panel;
	u32 refresh_rate;
	bool status;

	panel = sde_connector_panel(c_conn);
	if (!panel)
		return;

	mutex_lock(&panel->panel_lock);
	refresh_rate = panel->cur_mode->timing.refresh_rate;
	mutex_unlock(&panel->panel_lock);

	status = sde_connector_is_fod_enabled(c_conn);
	if (status == dsi_panel_get_fod_ui(panel))
		return;

	if (status && refresh_rate >= 120)
		sde_encoder_wait_for_event(c_conn->encoder, MSM_ENC_VBLANK);

	dsi_panel_set_fod_hbm(panel, status);

	dsi_panel_set_fod_ui(panel, status);
}

int sde_connector_pre_kickoff(struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;
	struct msm_display_kickoff_params params;
	struct dsi_display *display;
	int rc;

	if (!connector) {
		SDE_ERROR("invalid argument\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(connector->state);
	if (!c_conn->display) {
		SDE_ERROR("invalid connector display\n");
		return -EINVAL;
	}

	/*
	 * During pre kickoff DCS commands have to have an
	 * asynchronous wait to avoid an unnecessary stall
	 * in pre-kickoff. This flag must be reset at the
	 * end of display pre-kickoff.
	 */
	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DSI) {
		display = (struct dsi_display *)c_conn->display;
		display->queue_cmd_waits = true;
	}

	rc = _sde_connector_update_dirty_properties(connector);
	if (rc) {
		SDE_EVT32(connector->base.id, SDE_EVTLOG_ERROR);
		goto end;
	}

	if (!c_conn->ops.pre_kickoff)
		return 0;

	params.rois = &c_state->rois;
	params.hdr_meta = &c_state->hdr_meta;

	SDE_EVT32_VERBOSE(connector->base.id);

	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DSI)
		sde_connector_pre_update_fod_hbm(c_conn);

	rc = c_conn->ops.pre_kickoff(connector, c_conn->display, &params);

	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DSI)
		display->queue_cmd_waits = false;
end:
	return rc;
}

int sde_connector_prepare_commit(struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;
	struct msm_display_conn_params params;
	int rc;

	if (!connector) {
		SDE_ERROR("invalid argument\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(connector->state);
	if (!c_conn->display) {
		SDE_ERROR("invalid connector display\n");
		return -EINVAL;
	}

	if (!c_conn->ops.prepare_commit)
		return 0;

	memset(&params, 0, sizeof(params));

	if (c_conn->qsync_updated) {
		params.qsync_mode = c_conn->qsync_mode;
		params.qsync_update = true;
	}

	rc = c_conn->ops.prepare_commit(c_conn->display, &params);

	SDE_EVT32(connector->base.id, params.qsync_mode,
		  params.qsync_update, rc);

	return rc;
}

void sde_connector_helper_bridge_disable(struct drm_connector *connector)
{
	int rc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display;
	bool poms_pending = false;
	struct sde_kms *sde_kms;

	sde_kms = _sde_connector_get_kms(connector);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	c_conn = to_sde_connector(connector);
	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DSI) {
		display = (struct dsi_display *) c_conn->display;
		poms_pending = display->poms_pending;
	}

	if (!poms_pending) {
		rc = _sde_connector_update_dirty_properties(connector);
		if (rc) {
			SDE_ERROR("conn %d final pre kickoff failed %d\n",
					connector->base.id, rc);
			SDE_EVT32(connector->base.id, SDE_EVTLOG_ERROR);
		}
	}
	/* Disable ESD thread */
	sde_connector_schedule_status_work(connector, false);

	if (!sde_in_trusted_vm(sde_kms) && c_conn->bl_device) {
		c_conn->bl_device->props.power = FB_BLANK_POWERDOWN;
		c_conn->bl_device->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(c_conn->bl_device);
	}

	c_conn->allow_bl_update = false;
}

void sde_connector_helper_bridge_enable(struct drm_connector *connector)
{
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display;
	struct sde_kms *sde_kms;

	sde_kms = _sde_connector_get_kms(connector);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	c_conn = to_sde_connector(connector);
	display = (struct dsi_display *) c_conn->display;

	/*
	 * Special handling for some panels which need atleast
	 * one frame to be transferred to GRAM before enabling backlight.
	 * So delay backlight update to these panels until the
	 * first frame commit is received from the HW.
	 */
	if (display->panel->bl_config.bl_update ==
				BL_UPDATE_DELAY_UNTIL_FIRST_FRAME)
		sde_encoder_wait_for_event(c_conn->encoder,
				MSM_ENC_TX_COMPLETE);
	c_conn->allow_bl_update = true;

	if (!sde_in_trusted_vm(sde_kms) && c_conn->bl_device) {
		c_conn->bl_device->props.power = FB_BLANK_UNBLANK;
		c_conn->bl_device->props.state &= ~BL_CORE_FBBLANK;
		backlight_update_status(c_conn->bl_device);
	}
	c_conn->panel_dead = false;
}

int sde_connector_clk_ctrl(struct drm_connector *connector, bool enable)
{
	struct sde_connector *c_conn;
	struct dsi_display *display;
	u32 state = enable ? DSI_CLK_ON : DSI_CLK_OFF;
	int rc = 0;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	display = (struct dsi_display *) c_conn->display;

	if (display && c_conn->ops.clk_ctrl)
		rc = c_conn->ops.clk_ctrl(display->mdp_clk_handle,
				DSI_ALL_CLKS, state);

	return rc;
}

void sde_connector_destroy(struct drm_connector *connector)
{
	struct sde_connector *c_conn;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return;
	}

	c_conn = to_sde_connector(connector);

	if (c_conn->sysfs_dev)
		device_unregister(c_conn->sysfs_dev);

	/* cancel if any pending esd work */
	sde_connector_schedule_status_work(connector, false);

	if (c_conn->ops.pre_destroy)
		c_conn->ops.pre_destroy(connector, c_conn->display);

	if (c_conn->blob_caps)
		drm_property_blob_put(c_conn->blob_caps);
	if (c_conn->blob_hdr)
		drm_property_blob_put(c_conn->blob_hdr);
	if (c_conn->blob_dither)
		drm_property_blob_put(c_conn->blob_dither);
	if (c_conn->blob_mode_info)
		drm_property_blob_put(c_conn->blob_mode_info);
	if (c_conn->blob_ext_hdr)
		drm_property_blob_put(c_conn->blob_ext_hdr);

	if (c_conn->cdev)
		backlight_cdev_unregister(c_conn->cdev);
	if (c_conn->bl_device)
		backlight_device_unregister(c_conn->bl_device);
	drm_connector_unregister(connector);
	mutex_destroy(&c_conn->lock);
	sde_fence_deinit(c_conn->retire_fence);
	drm_connector_cleanup(connector);
	msm_property_destroy(&c_conn->property_info);
	kfree(c_conn);
}

/**
 * _sde_connector_destroy_fb - clean up connector state's out_fb buffer
 * @c_conn: Pointer to sde connector structure
 * @c_state: Pointer to sde connector state structure
 */
static void _sde_connector_destroy_fb(struct sde_connector *c_conn,
		struct sde_connector_state *c_state)
{
	if (!c_state || !c_state->out_fb) {
		SDE_ERROR("invalid state %pK\n", c_state);
		return;
	}

	drm_framebuffer_put(c_state->out_fb);
	c_state->out_fb = NULL;

	if (c_conn)
		c_state->property_values[CONNECTOR_PROP_OUT_FB].value =
			msm_property_get_default(&c_conn->property_info,
					CONNECTOR_PROP_OUT_FB);
	else
		c_state->property_values[CONNECTOR_PROP_OUT_FB].value = ~0;
}

static void sde_connector_atomic_destroy_state(struct drm_connector *connector,
		struct drm_connector_state *state)
{
	struct sde_connector *c_conn = NULL;
	struct sde_connector_state *c_state = NULL;

	if (!state) {
		SDE_ERROR("invalid state\n");
		return;
	}

	/*
	 * The base DRM framework currently always passes in a NULL
	 * connector pointer. This is not correct, but attempt to
	 * handle that case as much as possible.
	 */
	if (connector)
		c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(state);

	if (c_state->out_fb)
		_sde_connector_destroy_fb(c_conn, c_state);

	__drm_atomic_helper_connector_destroy_state(&c_state->base);

	if (!c_conn) {
		kfree(c_state);
	} else {
		/* destroy value helper */
		msm_property_destroy_state(&c_conn->property_info, c_state,
				&c_state->property_state);
	}
}

static void sde_connector_atomic_reset(struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return;
	}

	c_conn = to_sde_connector(connector);

	if (connector->state &&
			!sde_crtc_is_reset_required(connector->state->crtc)) {
		SDE_DEBUG_CONN(c_conn, "avoid reset for connector\n");
		return;
	}

	if (connector->state) {
		sde_connector_atomic_destroy_state(connector, connector->state);
		connector->state = 0;
	}

	c_state = msm_property_alloc_state(&c_conn->property_info);
	if (!c_state) {
		SDE_ERROR("state alloc failed\n");
		return;
	}

	/* reset value helper, zero out state structure and reset properties */
	msm_property_reset_state(&c_conn->property_info, c_state,
			&c_state->property_state,
			c_state->property_values);

	__drm_atomic_helper_connector_reset(connector, &c_state->base);
}

static struct drm_connector_state *
sde_connector_atomic_duplicate_state(struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state, *c_oldstate;

	if (!connector || !connector->state) {
		SDE_ERROR("invalid connector %pK\n", connector);
		return NULL;
	}

	c_conn = to_sde_connector(connector);
	c_oldstate = to_sde_connector_state(connector->state);
	c_state = msm_property_alloc_state(&c_conn->property_info);
	if (!c_state) {
		SDE_ERROR("state alloc failed\n");
		return NULL;
	}

	/* duplicate value helper */
	msm_property_duplicate_state(&c_conn->property_info,
			c_oldstate, c_state,
			&c_state->property_state, c_state->property_values);

	__drm_atomic_helper_connector_duplicate_state(connector,
			&c_state->base);

	/* additional handling for drm framebuffer objects */
	if (c_state->out_fb)
		drm_framebuffer_get(c_state->out_fb);

	/* clear dynamic HDR metadata from prev state */
	if (c_state->dyn_hdr_meta.dynamic_hdr_update) {
		c_state->dyn_hdr_meta.dynamic_hdr_update = false;
		c_state->dyn_hdr_meta.dynamic_hdr_payload_size = 0;
	}

	return &c_state->base;
}

int sde_connector_roi_v1_check_roi(struct drm_connector_state *conn_state)
{
	const struct msm_roi_alignment *align = NULL;
	struct sde_connector *c_conn = NULL;
	struct msm_mode_info mode_info;
	struct sde_connector_state *c_state;
	int i, w, h;

	if (!conn_state)
		return -EINVAL;

	memset(&mode_info, 0, sizeof(mode_info));

	c_state = to_sde_connector_state(conn_state);
	c_conn = to_sde_connector(conn_state->connector);

	memcpy(&mode_info, &c_state->mode_info, sizeof(c_state->mode_info));

	if (!mode_info.roi_caps.enabled)
		return 0;

	if (c_state->rois.num_rects > mode_info.roi_caps.num_roi) {
		SDE_ERROR_CONN(c_conn, "too many rects specified: %d > %d\n",
				c_state->rois.num_rects,
				mode_info.roi_caps.num_roi);
		return -E2BIG;
	}

	align = &mode_info.roi_caps.align;
	for (i = 0; i < c_state->rois.num_rects; ++i) {
		struct drm_clip_rect *roi_conn;

		roi_conn = &c_state->rois.roi[i];
		w = roi_conn->x2 - roi_conn->x1;
		h = roi_conn->y2 - roi_conn->y1;

		SDE_EVT32_VERBOSE(DRMID(&c_conn->base),
				roi_conn->x1, roi_conn->y1,
				roi_conn->x2, roi_conn->y2);

		if (w <= 0 || h <= 0) {
			SDE_ERROR_CONN(c_conn, "invalid conn roi w %d h %d\n",
					w, h);
			return -EINVAL;
		}

		if (w < align->min_width || w % align->width_pix_align) {
			SDE_ERROR_CONN(c_conn,
					"invalid conn roi width %d min %d align %d\n",
					w, align->min_width,
					align->width_pix_align);
			return -EINVAL;
		}

		if (h < align->min_height || h % align->height_pix_align) {
			SDE_ERROR_CONN(c_conn,
					"invalid conn roi height %d min %d align %d\n",
					h, align->min_height,
					align->height_pix_align);
			return -EINVAL;
		}

		if (roi_conn->x1 % align->xstart_pix_align) {
			SDE_ERROR_CONN(c_conn,
					"invalid conn roi x1 %d align %d\n",
					roi_conn->x1, align->xstart_pix_align);
			return -EINVAL;
		}

		if (roi_conn->y1 % align->ystart_pix_align) {
			SDE_ERROR_CONN(c_conn,
					"invalid conn roi y1 %d align %d\n",
					roi_conn->y1, align->ystart_pix_align);
			return -EINVAL;
		}
	}

	return 0;
}

static int _sde_connector_set_roi_v1(
		struct sde_connector *c_conn,
		struct sde_connector_state *c_state,
		void __user *usr_ptr)
{
	struct sde_drm_roi_v1 roi_v1;
	int i;

	if (!c_conn || !c_state) {
		SDE_ERROR("invalid args\n");
		return -EINVAL;
	}

	memset(&c_state->rois, 0, sizeof(c_state->rois));

	if (!usr_ptr) {
		SDE_DEBUG_CONN(c_conn, "rois cleared\n");
		return 0;
	}

	if (copy_from_user(&roi_v1, usr_ptr, sizeof(roi_v1))) {
		SDE_ERROR_CONN(c_conn, "failed to copy roi_v1 data\n");
		return -EINVAL;
	}

	SDE_DEBUG_CONN(c_conn, "num_rects %d\n", roi_v1.num_rects);

	if (roi_v1.num_rects == 0) {
		SDE_DEBUG_CONN(c_conn, "rois cleared\n");
		return 0;
	}

	if (roi_v1.num_rects > SDE_MAX_ROI_V1) {
		SDE_ERROR_CONN(c_conn, "num roi rects more than supported: %d",
				roi_v1.num_rects);
		return -EINVAL;
	}

	c_state->rois.num_rects = roi_v1.num_rects;
	for (i = 0; i < roi_v1.num_rects; ++i) {
		c_state->rois.roi[i] = roi_v1.roi[i];
		SDE_DEBUG_CONN(c_conn, "roi%d: roi (%d,%d) (%d,%d)\n", i,
				c_state->rois.roi[i].x1,
				c_state->rois.roi[i].y1,
				c_state->rois.roi[i].x2,
				c_state->rois.roi[i].y2);
	}

	return 0;
}

static int _sde_connector_set_ext_hdr_info(
	struct sde_connector *c_conn,
	struct sde_connector_state *c_state,
	void __user *usr_ptr)
{
	int rc = 0;
	struct drm_msm_ext_hdr_metadata *hdr_meta;
	size_t payload_size = 0;
	u8 *payload = NULL;
	int i;

	if (!c_conn || !c_state) {
		SDE_ERROR_CONN(c_conn, "invalid args\n");
		rc = -EINVAL;
		goto end;
	}

	memset(&c_state->hdr_meta, 0, sizeof(c_state->hdr_meta));

	if (!usr_ptr) {
		SDE_DEBUG_CONN(c_conn, "hdr metadata cleared\n");
		goto end;
	}

	if (!c_conn->hdr_supported) {
		SDE_ERROR_CONN(c_conn, "sink doesn't support HDR\n");
		rc = -ENOTSUPP;
		goto end;
	}

	if (copy_from_user(&c_state->hdr_meta,
		(void __user *)usr_ptr,
			sizeof(*hdr_meta))) {
		SDE_ERROR_CONN(c_conn, "failed to copy hdr metadata\n");
		rc = -EFAULT;
		goto end;
	}

	hdr_meta = &c_state->hdr_meta;

	/* dynamic metadata support */
	if (!hdr_meta->hdr_plus_payload_size || !hdr_meta->hdr_plus_payload)
		goto skip_dhdr;

	if (!c_conn->hdr_plus_app_ver) {
		SDE_ERROR_CONN(c_conn, "sink doesn't support dynamic HDR\n");
		rc = -ENOTSUPP;
		goto end;
	}

	payload_size = hdr_meta->hdr_plus_payload_size;
	if (payload_size > sizeof(c_state->dyn_hdr_meta.dynamic_hdr_payload)) {
		SDE_ERROR_CONN(c_conn, "payload size exceeds limit\n");
		rc = -EINVAL;
		goto end;
	}

	payload = c_state->dyn_hdr_meta.dynamic_hdr_payload;
	if (copy_from_user(payload,
			(void __user *)c_state->hdr_meta.hdr_plus_payload,
			payload_size)) {
		SDE_ERROR_CONN(c_conn, "failed to copy dhdr metadata\n");
		rc = -EFAULT;
		goto end;
	}

	/* verify 1st header byte, programmed in DP Infoframe SDP header */
	if (payload_size < 1 || (payload[0] != HDR10_PLUS_VSIF_TYPE_CODE)) {
		SDE_ERROR_CONN(c_conn, "invalid payload detected, size: %zd\n",
				payload_size);
		rc = -EINVAL;
		goto end;
	}

	c_state->dyn_hdr_meta.dynamic_hdr_update = true;

skip_dhdr:
	c_state->dyn_hdr_meta.dynamic_hdr_payload_size = payload_size;

	SDE_DEBUG_CONN(c_conn, "hdr_state %d\n", hdr_meta->hdr_state);
	SDE_DEBUG_CONN(c_conn, "hdr_supported %d\n", hdr_meta->hdr_supported);
	SDE_DEBUG_CONN(c_conn, "eotf %d\n", hdr_meta->eotf);
	SDE_DEBUG_CONN(c_conn, "white_point_x %d\n", hdr_meta->white_point_x);
	SDE_DEBUG_CONN(c_conn, "white_point_y %d\n", hdr_meta->white_point_y);
	SDE_DEBUG_CONN(c_conn, "max_luminance %d\n", hdr_meta->max_luminance);
	SDE_DEBUG_CONN(c_conn, "max_content_light_level %d\n",
				hdr_meta->max_content_light_level);
	SDE_DEBUG_CONN(c_conn, "max_average_light_level %d\n",
				hdr_meta->max_average_light_level);

	for (i = 0; i < HDR_PRIMARIES_COUNT; i++) {
		SDE_DEBUG_CONN(c_conn, "display_primaries_x [%d]\n",
				   hdr_meta->display_primaries_x[i]);
		SDE_DEBUG_CONN(c_conn, "display_primaries_y [%d]\n",
				   hdr_meta->display_primaries_y[i]);
	}
	SDE_DEBUG_CONN(c_conn, "hdr_plus payload%s updated, size %d\n",
			c_state->dyn_hdr_meta.dynamic_hdr_update ? "" : " NOT",
			c_state->dyn_hdr_meta.dynamic_hdr_payload_size);

end:
	return rc;
}

static int sde_connector_atomic_set_property(struct drm_connector *connector,
		struct drm_connector_state *state,
		struct drm_property *property,
		uint64_t val)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;
	int idx, rc;
	uint64_t fence_user_fd;
	uint64_t __user prev_user_fd;
	struct msm_param_info param_info;

	if (!connector || !state || !property) {
		SDE_ERROR("invalid argument(s), conn %pK, state %pK, prp %pK\n",
				connector, state, property);
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(state);

	/* generic property handling */
	rc = msm_property_atomic_set(&c_conn->property_info,
			&c_state->property_state, property, val);
	if (rc)
		goto end;

	/* connector-specific property handling */
	idx = msm_property_index(&c_conn->property_info, property);
	switch (idx) {
	case CONNECTOR_PROP_OUT_FB:
		/* clear old fb, if present */
		if (c_state->out_fb)
			_sde_connector_destroy_fb(c_conn, c_state);

		/* convert fb val to drm framebuffer and prepare it */
		c_state->out_fb =
			drm_framebuffer_lookup(connector->dev, NULL, val);
		if (!c_state->out_fb && val) {
			SDE_ERROR("failed to look up fb %lld\n", val);
			rc = -EFAULT;
		} else if (!c_state->out_fb && !val) {
			SDE_DEBUG("cleared fb_id\n");
			rc = 0;
		}
		break;
	case CONNECTOR_PROP_RETIRE_FENCE:
		if (!val)
			goto end;

		rc = copy_from_user(&prev_user_fd, (void __user *)val,
				sizeof(uint64_t));
		if (rc) {
			SDE_ERROR("copy from user failed rc:%d\n", rc);
			rc = -EFAULT;
			goto end;
		}

		/*
		 * client is expected to reset the property to -1 before
		 * requesting for the retire fence
		 */
		if (prev_user_fd == -1) {
			/*
			 * update the offset to a timeline for
			 * commit completion
			 */
			rc = sde_fence_create(c_conn->retire_fence,
						&fence_user_fd, 1);
			if (rc) {
				SDE_ERROR("fence create failed rc:%d\n", rc);
				goto end;
			}

			rc = copy_to_user((uint64_t __user *)(uintptr_t)val,
					&fence_user_fd, sizeof(uint64_t));
			if (rc) {
				SDE_ERROR("copy to user failed rc:%d\n", rc);
				/*
				 * fence will be released with timeline
				 * update
				 */
				put_unused_fd(fence_user_fd);
				rc = -EFAULT;
				goto end;
			}
		}
		break;
	case CONNECTOR_PROP_ROI_V1:
		rc = _sde_connector_set_roi_v1(c_conn, c_state,
				(void *)(uintptr_t)val);
		if (rc)
			SDE_ERROR_CONN(c_conn, "invalid roi_v1, rc: %d\n", rc);
		break;
	/* CONNECTOR_PROP_BL_SCALE and CONNECTOR_PROP_SV_BL_SCALE are
	 * color-processing properties. These two properties require
	 * special handling since they don't quite fit the current standard
	 * atomic set property framework.
	 */
	case CONNECTOR_PROP_BL_SCALE:
		c_conn->bl_scale = val;
		c_conn->bl_scale_dirty = true;
		break;
	case CONNECTOR_PROP_SV_BL_SCALE:
		c_conn->bl_scale_sv = val;
		c_conn->bl_scale_dirty = true;
		break;
	case CONNECTOR_PROP_HDR_METADATA:
		rc = _sde_connector_set_ext_hdr_info(c_conn,
			c_state, (void *)(uintptr_t)val);
		if (rc)
			SDE_ERROR_CONN(c_conn, "cannot set hdr info %d\n", rc);
		break;
	case CONNECTOR_PROP_QSYNC_MODE:
		msm_property_set_dirty(&c_conn->property_info,
				&c_state->property_state, idx);
		break;
	case CONNECTOR_PROP_HBM:
		param_info.value = val;
		param_info.param_idx = PARAM_HBM_ID;
		param_info.param_conn_idx = CONNECTOR_PROP_HBM;
		rc = _sde_connector_update_param(c_conn, &param_info);
		if (rc)
			goto end;
		break;
	case CONNECTOR_PROP_ACL:
		param_info.value = val;
		param_info.param_idx = PARAM_ACL_ID;
		param_info.param_conn_idx = CONNECTOR_PROP_ACL;
		rc = _sde_connector_update_param(c_conn, &param_info);
		if (rc)
			goto end;
		break;
	case CONNECTOR_PROP_CABC:
		param_info.value = val;
		param_info.param_idx = PARAM_CABC_ID;
		param_info.param_conn_idx = CONNECTOR_PROP_CABC;
		rc = _sde_connector_update_param(c_conn, &param_info);
		if (rc)
			goto end;
		break;
	case CONNECTOR_PROP_DC:
		param_info.value = val;
		param_info.param_idx = PARAM_DC_ID;
		param_info.param_conn_idx = CONNECTOR_PROP_DC;
		rc = _sde_connector_update_param(c_conn, &param_info);
		if (rc)
			goto end;
		break;
	case CONNECTOR_PROP_COLOR:
		param_info.value = val;
		param_info.param_idx = PARAM_COLOR_ID;
		param_info.param_conn_idx = CONNECTOR_PROP_COLOR;
		rc = _sde_connector_update_param(c_conn, &param_info);
		if (rc)
			goto end;
		break;
	default:
		break;
	}

	/* check for custom property handling */
	if (!rc && c_conn->ops.set_property) {
		rc = c_conn->ops.set_property(connector,
				state,
				idx,
				val,
				c_conn->display);

		/* potentially clean up out_fb if rc != 0 */
		if ((idx == CONNECTOR_PROP_OUT_FB) && rc)
			_sde_connector_destroy_fb(c_conn, c_state);
	}
end:
	return rc;
}

static int sde_connector_atomic_get_property(struct drm_connector *connector,
		const struct drm_connector_state *state,
		struct drm_property *property,
		uint64_t *val)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;
	int idx, rc = -EINVAL;

	if (!connector || !state) {
		SDE_ERROR("invalid argument(s), conn %pK, state %pK\n",
				connector, state);
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(state);

	idx = msm_property_index(&c_conn->property_info, property);
	if (idx == CONNECTOR_PROP_RETIRE_FENCE) {
		*val = ~0;
		rc = 0;
	} else {
		/* get cached property value */
		rc = msm_property_atomic_get(&c_conn->property_info,
				&c_state->property_state, property, val);
	}

	/* allow for custom override */
	if (c_conn->ops.get_property)
		rc = c_conn->ops.get_property(connector,
				(struct drm_connector_state *)state,
				idx,
				val,
				c_conn->display);
	return rc;
}

void sde_conn_timeline_status(struct drm_connector *conn)
{
	struct sde_connector *c_conn;

	if (!conn) {
		SDE_ERROR("invalid connector\n");
		return;
	}

	c_conn = to_sde_connector(conn);
	sde_fence_timeline_status(c_conn->retire_fence, &conn->base);
}

void sde_connector_prepare_fence(struct drm_connector *connector)
{
	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return;
	}

	sde_fence_prepare(to_sde_connector(connector)->retire_fence);
}

void sde_connector_complete_commit(struct drm_connector *connector,
		ktime_t ts, enum sde_fence_event fence_event)
{
	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return;
	}

	/* signal connector's retire fence */
	sde_fence_signal(to_sde_connector(connector)->retire_fence,
			ts, fence_event);
}

void sde_connector_commit_reset(struct drm_connector *connector, ktime_t ts)
{
	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return;
	}

	/* signal connector's retire fence */
	sde_fence_signal(to_sde_connector(connector)->retire_fence,
			ts, SDE_FENCE_RESET_TIMELINE);
}

static void sde_connector_update_hdr_props(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct drm_msm_ext_hdr_properties hdr = {0};

	hdr.hdr_metadata_type_one = c_conn->hdr_metadata_type_one ? 1 : 0;
	hdr.hdr_supported = c_conn->hdr_supported ? 1 : 0;
	hdr.hdr_eotf = c_conn->hdr_eotf;
	hdr.hdr_max_luminance = c_conn->hdr_max_luminance;
	hdr.hdr_avg_luminance = c_conn->hdr_avg_luminance;
	hdr.hdr_min_luminance = c_conn->hdr_min_luminance;
	hdr.hdr_plus_supported = c_conn->hdr_plus_app_ver;

	msm_property_set_blob(&c_conn->property_info, &c_conn->blob_ext_hdr,
			&hdr, sizeof(hdr), CONNECTOR_PROP_EXT_HDR_INFO);
}

static void sde_connector_update_colorspace(struct drm_connector *connector)
{
	int ret;
	struct sde_connector *c_conn = to_sde_connector(connector);

	ret = msm_property_set_property(
			sde_connector_get_propinfo(connector),
			sde_connector_get_property_state(connector->state),
			CONNECTOR_PROP_SUPPORTED_COLORSPACES,
				c_conn->color_enc_fmt);

	if (ret)
		SDE_ERROR("failed to set colorspace property for connector\n");
}

static enum drm_connector_status
sde_connector_detect(struct drm_connector *connector, bool force)
{
	enum drm_connector_status status = connector_status_unknown;
	struct sde_connector *c_conn;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return status;
	}

	c_conn = to_sde_connector(connector);

	if (c_conn->ops.detect)
		status = c_conn->ops.detect(connector,
				force,
				c_conn->display);

	return status;
}

int sde_connector_get_dpms(struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	int rc;

	if (!connector) {
		SDE_DEBUG("invalid connector\n");
		return DRM_MODE_DPMS_OFF;
	}

	c_conn = to_sde_connector(connector);

	mutex_lock(&c_conn->lock);
	rc = c_conn->dpms_mode;
	mutex_unlock(&c_conn->lock);

	return rc;
}

int sde_connector_set_property_for_commit(struct drm_connector *connector,
		struct drm_atomic_state *atomic_state,
		uint32_t property_idx, uint64_t value)
{
	struct drm_connector_state *state;
	struct drm_property *property;
	struct sde_connector *c_conn;

	if (!connector || !atomic_state) {
		SDE_ERROR("invalid argument(s), conn %d, state %d\n",
				connector != NULL, atomic_state != NULL);
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	property = msm_property_index_to_drm_property(
			&c_conn->property_info, property_idx);
	if (!property) {
		SDE_ERROR("invalid property index %d\n", property_idx);
		return -EINVAL;
	}

	state = drm_atomic_get_connector_state(atomic_state, connector);
	if (IS_ERR_OR_NULL(state)) {
		SDE_ERROR("failed to get conn %d state\n",
				connector->base.id);
		return -EINVAL;
	}

	return sde_connector_atomic_set_property(
			connector, state, property, value);
}

int sde_connector_helper_reset_custom_properties(
		struct drm_connector *connector,
		struct drm_connector_state *connector_state)
{
	struct sde_connector *c_conn;
	struct sde_connector_state *c_state;
	struct drm_property *drm_prop;
	enum msm_mdp_conn_property prop_idx;

	if (!connector || !connector_state) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	c_state = to_sde_connector_state(connector_state);

	for (prop_idx = 0; prop_idx < CONNECTOR_PROP_COUNT; prop_idx++) {
		uint64_t val = c_state->property_values[prop_idx].value;
		uint64_t def;
		int ret;

		drm_prop = msm_property_index_to_drm_property(
				&c_conn->property_info, prop_idx);
		if (!drm_prop) {
			/* not all props will be installed, based on caps */
			SDE_DEBUG_CONN(c_conn, "invalid property index %d\n",
					prop_idx);
			continue;
		}

		def = msm_property_get_default(&c_conn->property_info,
				prop_idx);
		if (val == def)
			continue;

		SDE_DEBUG_CONN(c_conn, "set prop %s idx %d from %llu to %llu\n",
				drm_prop->name, prop_idx, val, def);

		ret = sde_connector_atomic_set_property(connector,
				connector_state, drm_prop, def);
		if (ret) {
			SDE_ERROR_CONN(c_conn,
					"set property failed, idx %d ret %d\n",
					prop_idx, ret);
			continue;
		}
	}

	return 0;
}

static int _sde_connector_lm_preference(struct sde_connector *sde_conn,
		 struct sde_kms *sde_kms, uint32_t disp_type)
{
	int ret = 0;
	u32 num_lm = 0;

	if (!sde_conn || !sde_kms || !sde_conn->ops.get_default_lms) {
		SDE_DEBUG("invalid input params");
		return -EINVAL;
	}

	if (!disp_type || disp_type >= SDE_CONNECTOR_MAX) {
		SDE_DEBUG("invalid display_type");
		return -EINVAL;
	}

	ret = sde_conn->ops.get_default_lms(sde_conn->display, &num_lm);
	if (ret || !num_lm) {
		SDE_DEBUG("failed to get default lm count");
		return ret;
	}

	if (num_lm > sde_kms->catalog->mixer_count) {
		SDE_DEBUG(
				"topology requesting more lms [%d] than hw exists [%d]",
				num_lm, sde_kms->catalog->mixer_count);
		return -EINVAL;
	}

	sde_conn->lm_mask = sde_hw_mixer_set_preference(sde_kms->catalog, num_lm, disp_type);

	return ret;
}

int sde_connector_get_panel_vfp(struct drm_connector *connector,
	struct drm_display_mode *mode)
{
	struct sde_connector *c_conn;
	int vfp = -EINVAL;

	if (!connector || !mode) {
		SDE_ERROR("invalid connector\n");
		return vfp;
	}
	c_conn = to_sde_connector(connector);
	if (!c_conn->ops.get_panel_vfp)
		return vfp;

	vfp = c_conn->ops.get_panel_vfp(c_conn->display,
		mode->hdisplay, mode->vdisplay);
	if (vfp <= 0)
		SDE_ERROR("Failed get_panel_vfp %d\n", vfp);

	return vfp;
}

static int _sde_debugfs_conn_cmd_tx_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->private_data = inode->i_private;
	return nonseekable_open(inode, file);
}

static ssize_t _sde_debugfs_conn_cmd_tx_sts_read(struct file *file,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn = NULL;
	char buffer[MAX_CMD_PAYLOAD_SIZE] = {0};
	int blen = 0;

	if (*ppos)
		return 0;

	if (!connector) {
		SDE_ERROR("invalid argument, conn is NULL\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);

	mutex_lock(&c_conn->lock);
	blen = snprintf(buffer, MAX_CMD_PAYLOAD_SIZE,
		"last_cmd_tx_sts:0x%x",
		c_conn->last_cmd_tx_sts);
	mutex_unlock(&c_conn->lock);

	SDE_DEBUG("output: %s\n", buffer);
	if (blen <= 0) {
		SDE_ERROR("snprintf failed, blen %d\n", blen);
		return -EINVAL;
	}

	if (blen > count)
		blen = count;

	blen = min_t(size_t, blen, MAX_CMD_PAYLOAD_SIZE);
	if (copy_to_user(buf, buffer, blen)) {
		SDE_ERROR("copy to user buffer failed\n");
		return -EFAULT;
	}

	*ppos += blen;
	return blen;
}

static ssize_t _sde_debugfs_conn_cmd_tx_write(struct file *file,
			const char __user *p, size_t count, loff_t *ppos)
{
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn = NULL;
	struct sde_vm_ops *vm_ops;
	struct sde_kms *sde_kms;
	char *input, *token, *input_copy, *input_dup = NULL;
	const char *delim = " ";
	char buffer[MAX_CMD_PAYLOAD_SIZE] = {0};
	int rc = 0, strtoint = 0;
	u32 buf_size = 0;

	if (*ppos || !connector) {
		SDE_ERROR("invalid argument(s), conn %d\n", connector != NULL);
		return -EINVAL;
	}
	c_conn = to_sde_connector(connector);

	sde_kms = _sde_connector_get_kms(&c_conn->base);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return -EINVAL;
	}

	if (!c_conn->ops.cmd_transfer) {
		SDE_ERROR("no cmd transfer support for connector name %s\n",
				c_conn->name);
		return -EINVAL;
	}

	input = kzalloc(count + 1, GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	vm_ops = sde_vm_get_ops(sde_kms);
	sde_vm_lock(sde_kms);
	if (vm_ops && vm_ops->vm_owns_hw && !vm_ops->vm_owns_hw(sde_kms)) {
		SDE_DEBUG("op not supported due to HW unavailablity\n");
		rc = -EOPNOTSUPP;
		goto end;
	}

	if (copy_from_user(input, p, count)) {
		SDE_ERROR("copy from user failed\n");
		rc = -EFAULT;
		goto end;
	}
	input[count] = '\0';

	SDE_INFO("Command requested for transfer to panel: %s\n", input);

	input_copy = kstrdup(input, GFP_KERNEL);
	if (!input_copy) {
		rc = -ENOMEM;
		goto end;
	}

	input_dup = input_copy;
	token = strsep(&input_copy, delim);
	while (token) {
		rc = kstrtoint(token, 0, &strtoint);
		if (rc) {
			SDE_ERROR("input buffer conversion failed\n");
			goto end1;
		}

		buffer[buf_size++] = (strtoint & 0xff);
		if (buf_size >= MAX_CMD_PAYLOAD_SIZE) {
			SDE_ERROR("buffer size exceeding the limit %d\n",
					MAX_CMD_PAYLOAD_SIZE);
			rc = -EFAULT;
			goto end1;
		}
		token = strsep(&input_copy, delim);
	}
	SDE_DEBUG("command packet size in bytes: %u\n", buf_size);
	if (!buf_size) {
		rc = -EFAULT;
		goto end1;
	}

	mutex_lock(&c_conn->lock);
	rc = c_conn->ops.cmd_transfer(&c_conn->base, c_conn->display, buffer,
			buf_size);
	c_conn->last_cmd_tx_sts = !rc ? true : false;
	mutex_unlock(&c_conn->lock);

	rc = count;
end1:
	kfree(input_dup);
end:
	sde_vm_unlock(sde_kms);
	kfree(input);
	return rc;
}

static const struct file_operations conn_cmd_tx_fops = {
	.open =		_sde_debugfs_conn_cmd_tx_open,
	.read =		_sde_debugfs_conn_cmd_tx_sts_read,
	.write =	_sde_debugfs_conn_cmd_tx_write,
};

static int _sde_debugfs_conn_cmd_rx_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->private_data = inode->i_private;
	return nonseekable_open(inode, file);
}

static ssize_t _sde_debugfs_conn_cmd_rx_read(struct file *file,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn = NULL;
	char *strs = NULL;
	char *strs_temp = NULL;
	int blen = 0, i = 0, n = 0, left_size = 0;

	if (*ppos)
		return 0;

	if (!connector) {
		SDE_ERROR("invalid argument, conn is NULL\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (c_conn->rx_len <= 0 || c_conn->rx_len > MAX_CMD_RECEIVE_SIZE) {
		SDE_ERROR("no valid data from panel\n");
		return -EINVAL;
	}

	/*
	 * Rx data was stored as HEX value in rx buffer,
	 * convert 1 HEX value to strings for display, need 5 bytes.
	 * for example: HEX value 0xFF, converted to strings, should be '0',
	 * 'x','F','F' and 1 space.
	 */
	left_size = c_conn->rx_len * 5 + 1;
	strs = kzalloc(left_size, GFP_KERNEL);
	if (!strs)
		return -ENOMEM;
	strs_temp = strs;

	mutex_lock(&c_conn->lock);
	for (i = 0; i < c_conn->rx_len; i++) {
		n = scnprintf(strs_temp, left_size, "0x%.2x ",
			     c_conn->cmd_rx_buf[i]);
		strs_temp += n;
		left_size -= n;
	}
	mutex_unlock(&c_conn->lock);

	blen = strlen(strs);
	if (blen <= 0) {
		SDE_ERROR("snprintf failed, blen %d\n", blen);
		blen = -EFAULT;
		goto err;
	}

	if (copy_to_user(buf, strs, blen)) {
		SDE_ERROR("copy to user buffer failed\n");
		blen = -EFAULT;
		goto err;
	}

	*ppos += blen;

err:
	kfree(strs);
	return blen;
}


static ssize_t _sde_debugfs_conn_cmd_rx_write(struct file *file,
			const char __user *p, size_t count, loff_t *ppos)
{
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn = NULL;
	char *input, *token, *input_copy, *input_dup = NULL;
	const char *delim = " ";
	unsigned char buffer[MAX_CMD_PAYLOAD_SIZE] = {0};
	int rc = 0, strtoint = 0;
	u32 buf_size = 0;

	if (*ppos || !connector) {
		SDE_ERROR("invalid argument(s), conn %d\n", connector != NULL);
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (!c_conn->ops.cmd_receive) {
		SDE_ERROR("no cmd receive support for connector name %s\n",
				c_conn->name);
		return -EINVAL;
	}

	memset(c_conn->cmd_rx_buf, 0x0, MAX_CMD_RECEIVE_SIZE);
	c_conn->rx_len = 0;

	input = kzalloc(count + 1, GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	if (copy_from_user(input, p, count)) {
		SDE_ERROR("copy from user failed\n");
		rc  = -EFAULT;
		goto end;
	}
	input[count] = '\0';

	SDE_INFO("Command requested for rx from panel: %s\n", input);

	input_copy = kstrdup(input, GFP_KERNEL);
	if (!input_copy) {
		rc = -ENOMEM;
		goto end;
	}

	input_dup = input_copy;
	token = strsep(&input_copy, delim);
	while (token) {
		rc = kstrtoint(token, 0, &strtoint);
		if (rc) {
			SDE_ERROR("input buffer conversion failed\n");
			goto end1;
		}

		buffer[buf_size++] = (strtoint & 0xff);
		if (buf_size >= MAX_CMD_PAYLOAD_SIZE) {
			SDE_ERROR("buffer size = %d exceeding the limit %d\n",
					buf_size, MAX_CMD_PAYLOAD_SIZE);
			rc = -EFAULT;
			goto end1;
		}
		token = strsep(&input_copy, delim);
	}

	if (!buffer[0] || buffer[0] > MAX_CMD_RECEIVE_SIZE) {
		SDE_ERROR("invalid rx length\n");
		rc = -EFAULT;
		goto end1;
	}

	SDE_DEBUG("command packet size in bytes: %u, rx len: %u\n",
			buf_size, buffer[0]);
	if (!buf_size) {
		rc = -EFAULT;
		goto end1;
	}

	mutex_lock(&c_conn->lock);
	c_conn->rx_len = c_conn->ops.cmd_receive(c_conn->display, buffer + 1,
			buf_size - 1, c_conn->cmd_rx_buf, buffer[0]);
	mutex_unlock(&c_conn->lock);

	if (c_conn->rx_len <= 0)
		rc = -EINVAL;
	else
		rc = count;
end1:
	kfree(input_dup);
end:
	kfree(input);
	return rc;
}

static const struct file_operations conn_cmd_rx_fops = {
	.open =         _sde_debugfs_conn_cmd_rx_open,
	.read =         _sde_debugfs_conn_cmd_rx_read,
	.write =        _sde_debugfs_conn_cmd_rx_write,
};

#ifdef CONFIG_DEBUG_FS
/**
 * sde_connector_init_debugfs - initialize connector debugfs
 * @connector: Pointer to drm connector
 */
static int sde_connector_init_debugfs(struct drm_connector *connector)
{
	struct sde_connector *sde_connector;
	struct msm_display_info info;

	if (!connector || !connector->debugfs_entry) {
		SDE_ERROR("invalid connector\n");
		return -EINVAL;
	}

	sde_connector = to_sde_connector(connector);

	sde_connector_get_info(connector, &info);
	if (sde_connector->ops.check_status &&
		(info.capabilities & MSM_DISPLAY_ESD_ENABLED)) {
		debugfs_create_u32("esd_status_interval", 0600,
				connector->debugfs_entry,
				&sde_connector->esd_status_interval);
	}

	if (sde_connector->ops.cmd_transfer) {
		if (!debugfs_create_file("tx_cmd", 0600,
			connector->debugfs_entry,
			connector, &conn_cmd_tx_fops)) {
			SDE_ERROR("failed to create connector cmd_tx\n");
			return -ENOMEM;
		}
	}

	if (sde_connector->ops.cmd_receive) {
		if (!debugfs_create_file("rx_cmd", 0600,
			connector->debugfs_entry,
			connector, &conn_cmd_rx_fops)) {
			SDE_ERROR("failed to create connector cmd_rx\n");
			return -ENOMEM;
		}
	}

	return 0;
}
#else
static int sde_connector_init_debugfs(struct drm_connector *connector)
{
	return 0;
}
#endif

static int sde_connector_late_register(struct drm_connector *connector)
{
	return sde_connector_init_debugfs(connector);
}

static void sde_connector_early_unregister(struct drm_connector *connector)
{
	/* debugfs under connector->debugfs are deleted by drm_debugfs */
}

static int sde_connector_fill_modes(struct drm_connector *connector,
		uint32_t max_width, uint32_t max_height)
{
	int rc, mode_count = 0;
	struct sde_connector *sde_conn = NULL;

	sde_conn = to_sde_connector(connector);
	if (!sde_conn) {
		SDE_ERROR("invalid arguments\n");
		return 0;
	}

	mode_count = drm_helper_probe_single_connector_modes(connector,
			max_width, max_height);

	if (sde_conn->ops.set_allowed_mode_switch)
		sde_conn->ops.set_allowed_mode_switch(connector,
				sde_conn->display);

	rc = sde_connector_set_blob_data(connector,
				connector->state,
				CONNECTOR_PROP_MODE_INFO);
	if (rc) {
		SDE_ERROR_CONN(sde_conn,
			"failed to setup mode info prop, rc = %d\n", rc);
		return 0;
	}

	return mode_count;
}

static const struct drm_connector_funcs sde_connector_ops = {
	.reset =                  sde_connector_atomic_reset,
	.detect =                 sde_connector_detect,
	.destroy =                sde_connector_destroy,
	.fill_modes =             sde_connector_fill_modes,
	.atomic_duplicate_state = sde_connector_atomic_duplicate_state,
	.atomic_destroy_state =   sde_connector_atomic_destroy_state,
	.atomic_set_property =    sde_connector_atomic_set_property,
	.atomic_get_property =    sde_connector_atomic_get_property,
	.late_register =          sde_connector_late_register,
	.early_unregister =       sde_connector_early_unregister,
};

static int sde_connector_get_modes(struct drm_connector *connector)
{
	struct sde_connector *c_conn;
	struct msm_resource_caps_info avail_res;
	int mode_count = 0;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return 0;
	}

	c_conn = to_sde_connector(connector);
	if (!c_conn->ops.get_modes) {
		SDE_DEBUG("missing get_modes callback\n");
		return 0;
	}

	memset(&avail_res, 0, sizeof(avail_res));
	sde_connector_get_avail_res_info(connector, &avail_res);

	mode_count = c_conn->ops.get_modes(connector, c_conn->display,
			&avail_res);
	if (!mode_count) {
		SDE_ERROR_CONN(c_conn, "failed to get modes\n");
		return 0;
	}

	if (c_conn->hdr_capable)
		sde_connector_update_hdr_props(connector);

	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DisplayPort)
		sde_connector_update_colorspace(connector);

	return mode_count;
}

static enum drm_mode_status
sde_connector_mode_valid(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	struct sde_connector *c_conn;
	struct msm_resource_caps_info avail_res;

	if (!connector || !mode) {
		SDE_ERROR("invalid argument(s), conn %pK, mode %pK\n",
				connector, mode);
		return MODE_ERROR;
	}

	c_conn = to_sde_connector(connector);

	memset(&avail_res, 0, sizeof(avail_res));
	sde_connector_get_avail_res_info(connector, &avail_res);

	if (c_conn->ops.mode_valid)
		return c_conn->ops.mode_valid(connector, mode, c_conn->display,
				&avail_res);

	/* assume all modes okay by default */
	return MODE_OK;
}

static struct drm_encoder *
sde_connector_best_encoder(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return NULL;
	}

	/*
	 * This is true for now, revisit this code when multiple encoders are
	 * supported.
	 */
	return c_conn->encoder;
}

static struct drm_encoder *
sde_connector_atomic_best_encoder(struct drm_connector *connector,
		struct drm_connector_state *connector_state)
{
	struct sde_connector *c_conn;
	struct drm_encoder *encoder = NULL;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return NULL;
	}

	c_conn = to_sde_connector(connector);

	if (c_conn->ops.atomic_best_encoder)
		encoder = c_conn->ops.atomic_best_encoder(connector,
				c_conn->display, connector_state);

	return encoder;
}

static int sde_connector_atomic_check(struct drm_connector *connector,
		struct drm_atomic_state *state)
{
	struct sde_connector *c_conn;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	if (c_conn->ops.atomic_check)
		return c_conn->ops.atomic_check(connector,
				c_conn->display, state);

	return 0;
}

static void _sde_connector_report_panel_dead(struct sde_connector *conn,
	bool skip_pre_kickoff)
{
	struct drm_event event;

	if (!conn)
		return;

	/* Panel dead notification can come:
	 * 1) ESD thread
	 * 2) Commit thread (if TE stops coming)
	 * So such case, avoid failure notification twice.
	 */
	if (conn->panel_dead)
		return;

	SDE_EVT32(SDE_EVTLOG_ERROR);
	sde_encoder_display_failure_notification(conn->encoder,
		skip_pre_kickoff);

	conn->panel_dead = true;
	event.type = DRM_EVENT_PANEL_DEAD;
	event.length = sizeof(bool);
	msm_mode_object_event_notify(&conn->base.base,
		conn->base.dev, &event, (u8 *)&conn->panel_dead);
	SDE_ERROR("esd check failed report PANEL_DEAD conn_id: %d enc_id: %d\n",
			conn->base.base.id, conn->encoder->base.id);
}

int sde_connector_esd_status(struct drm_connector *conn)
{
	struct sde_connector *sde_conn = NULL;
	struct dsi_display *display;
	int ret = 0;

	if (!conn)
		return ret;

	sde_conn = to_sde_connector(conn);
	if (!sde_conn || !sde_conn->ops.check_status)
		return ret;

	display = sde_conn->display;

	/* protect this call with ESD status check call */
	mutex_lock(&sde_conn->lock);
	if (atomic_read(&(display->panel->esd_recovery_pending))) {
		SDE_ERROR("ESD recovery already pending\n");
		mutex_unlock(&sde_conn->lock);
		return -ETIMEDOUT;
	}
	ret = sde_conn->ops.check_status(&sde_conn->base,
					 sde_conn->display, true);
	mutex_unlock(&sde_conn->lock);

	if (ret <= 0) {
		/* cancel if any pending esd work */
		sde_connector_schedule_status_work(conn, false);
		_sde_connector_report_panel_dead(sde_conn, true);
		ret = -ETIMEDOUT;
	} else {
		SDE_DEBUG("Successfully received TE from panel\n");
		ret = 0;
	}
	SDE_EVT32(ret);

	return ret;
}

static void sde_connector_check_status_work(struct work_struct *work)
{
	struct sde_connector *conn;
	int rc = 0;
	struct device *dev;

	conn = container_of(to_delayed_work(work),
			struct sde_connector, status_work);
	if (!conn) {
		SDE_ERROR("not able to get connector object\n");
		return;
	}

	mutex_lock(&conn->lock);
	dev = conn->base.dev->dev;

	if (!conn->ops.check_status || dev->power.is_suspended ||
			(conn->dpms_mode != DRM_MODE_DPMS_ON)) {
		SDE_DEBUG("dpms mode: %d\n", conn->dpms_mode);
		mutex_unlock(&conn->lock);
		return;
	}

	rc = conn->ops.check_status(&conn->base, conn->display, false);
	mutex_unlock(&conn->lock);

	if (rc > 0) {
		u32 interval;

		SDE_DEBUG("esd check status success conn_id: %d enc_id: %d\n",
				conn->base.base.id, conn->encoder->base.id);

		/* If debugfs property is not set then take default value */
		interval = conn->esd_status_interval ?
			conn->esd_status_interval : STATUS_CHECK_INTERVAL_MS;
		schedule_delayed_work(&conn->status_work,
			msecs_to_jiffies(interval));
		return;
	}

	_sde_connector_report_panel_dead(conn, false);
}

static const struct drm_connector_helper_funcs sde_connector_helper_ops = {
	.get_modes =    sde_connector_get_modes,
	.mode_valid =   sde_connector_mode_valid,
	.best_encoder = sde_connector_best_encoder,
	.atomic_check = sde_connector_atomic_check,
};

static const struct drm_connector_helper_funcs sde_connector_helper_ops_v2 = {
	.get_modes =    sde_connector_get_modes,
	.mode_valid =   sde_connector_mode_valid,
	.best_encoder = sde_connector_best_encoder,
	.atomic_best_encoder = sde_connector_atomic_best_encoder,
	.atomic_check = sde_connector_atomic_check,
};

static int sde_connector_populate_mode_info(struct drm_connector *conn,
	struct sde_kms_info *info)
{
	struct sde_kms *sde_kms;
	struct sde_connector *c_conn = NULL;
	struct drm_display_mode *mode;
	struct msm_mode_info mode_info;
	int rc = 0;

	sde_kms = _sde_connector_get_kms(conn);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(conn);
	if (!c_conn->ops.get_mode_info) {
		SDE_ERROR_CONN(c_conn, "get_mode_info not defined\n");
		return -EINVAL;
	}

	list_for_each_entry(mode, &conn->modes, head) {
		int topology_idx = 0;

		memset(&mode_info, 0, sizeof(mode_info));

		rc = sde_connector_get_mode_info(&c_conn->base, mode,
				&mode_info);
		if (rc) {
			SDE_ERROR_CONN(c_conn,
				"failed to get mode info for mode %s\n",
				mode->name);
			continue;
		}

		sde_kms_info_add_keystr(info, "mode_name", mode->name);

		sde_kms_info_add_keyint(info, "bit_clk_rate",
					mode_info.clk_rate);

		topology_idx = (int)sde_rm_get_topology_name(&sde_kms->rm,
					mode_info.topology);
		if (topology_idx < SDE_RM_TOPOLOGY_MAX) {
			sde_kms_info_add_keystr(info, "topology",
					e_topology_name[topology_idx].name);
		} else {
			SDE_ERROR_CONN(c_conn, "invalid topology\n");
			continue;
		}

		sde_kms_info_add_keyint(info, "has_cwb_crop", sde_kms->catalog->has_cwb_crop);

		sde_kms_info_add_keyint(info, "mdp_transfer_time_us",
			mode_info.mdp_transfer_time_us);

		sde_kms_info_add_keyint(info, "allowed_mode_switch",
			mode_info.allowed_mode_switches);

		if (!mode_info.roi_caps.num_roi)
			continue;

		sde_kms_info_add_keyint(info, "partial_update_num_roi",
			mode_info.roi_caps.num_roi);
		sde_kms_info_add_keyint(info, "partial_update_xstart",
			mode_info.roi_caps.align.xstart_pix_align);
		sde_kms_info_add_keyint(info, "partial_update_walign",
			mode_info.roi_caps.align.width_pix_align);
		sde_kms_info_add_keyint(info, "partial_update_wmin",
			mode_info.roi_caps.align.min_width);
		sde_kms_info_add_keyint(info, "partial_update_ystart",
			mode_info.roi_caps.align.ystart_pix_align);
		sde_kms_info_add_keyint(info, "partial_update_halign",
			mode_info.roi_caps.align.height_pix_align);
		sde_kms_info_add_keyint(info, "partial_update_hmin",
			mode_info.roi_caps.align.min_height);
		sde_kms_info_add_keyint(info, "partial_update_roimerge",
			mode_info.roi_caps.merge_rois);
	}

	return rc;
}

static int sde_connector_install_panel_params(struct sde_connector *c_conn)
{
	struct panel_param *param_cmds;
	uint32_t prop_idx;
	int i;
	struct dsi_display *dsi_display;
	u16 prop_max, prop_min, prop_init;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	dsi_display = (struct dsi_display *) (c_conn->display);
	param_cmds = dsi_display->panel->param_cmds;
	for (i = 0; i < PARAM_ID_NUM; i++) {
		SDE_DEBUG("%s:i = %d param_name = %s is_support=%d\n",
			__func__, i,
                        param_cmds->param_name, param_cmds->is_supported);

		if (!strncmp(param_cmds->param_name, "HBM", 3))
			prop_idx = CONNECTOR_PROP_HBM;
		else if (!strncmp(param_cmds->param_name, "CABC", 4))
			prop_idx = CONNECTOR_PROP_CABC;
		else if (!strncmp(param_cmds->param_name, "ACL", 3))
			prop_idx = CONNECTOR_PROP_ACL;
		else if (!strncmp(param_cmds->param_name, "DC", 2))
			prop_idx = CONNECTOR_PROP_DC;
		else if (!strncmp(param_cmds->param_name, "COLOR", 5))
			prop_idx = CONNECTOR_PROP_COLOR;
		else {
			SDE_ERROR("Invalid param_name =%s\n",
						param_cmds->param_name);
			return -EINVAL;
		}

		if (param_cmds->is_supported) {
			prop_max = param_cmds->val_max;
			prop_min = PARAM_STATE_OFF;
			prop_init = param_cmds->default_value;
		} else {
			prop_max = PARAM_STATE_OFF;
			prop_min = PARAM_STATE_OFF;
			prop_init = PARAM_STATE_DISABLE;
		}

		msm_property_install_volatile_range( &c_conn->property_info,
					param_cmds->param_name, 0x0,
					prop_min, prop_max,
					prop_init, prop_idx);

		param_cmds++;
	}

	return 0;
}

int sde_connector_set_blob_data(struct drm_connector *conn,
		struct drm_connector_state *state,
		enum msm_mdp_conn_property prop_id)
{
	struct sde_kms_info *info;
	struct sde_connector *c_conn = NULL;
	struct sde_connector_state *sde_conn_state = NULL;
	struct msm_mode_info mode_info;
	struct drm_property_blob **blob = NULL;
	int rc = 0;

	c_conn = to_sde_connector(conn);
	if (!c_conn) {
		SDE_ERROR("invalid argument\n");
		return -EINVAL;
	}

	info = vzalloc(sizeof(*info));
	if (!info)
		return -ENOMEM;

	sde_kms_info_reset(info);

	switch (prop_id) {
	case CONNECTOR_PROP_SDE_INFO:
		memset(&mode_info, 0, sizeof(mode_info));

		if (state) {
			sde_conn_state = to_sde_connector_state(state);
			memcpy(&mode_info, &sde_conn_state->mode_info,
					sizeof(sde_conn_state->mode_info));
		} else {
			/**
			 * connector state is assigned only on first
			 * atomic_commit. But this function is allowed to be
			 * invoked during probe/init sequence. So not throwing
			 * an error.
			 */
			SDE_DEBUG_CONN(c_conn, "invalid connector state\n");
		}

		if (c_conn->ops.set_info_blob) {
			rc = c_conn->ops.set_info_blob(conn, info,
					c_conn->display, &mode_info);
			if (rc) {
				SDE_ERROR_CONN(c_conn,
						"set_info_blob failed, %d\n",
						rc);
				goto exit;
			}
		}

		blob = &c_conn->blob_caps;
	break;
	case CONNECTOR_PROP_MODE_INFO:
		rc = sde_connector_populate_mode_info(conn, info);
		if (rc) {
			SDE_ERROR_CONN(c_conn,
					"mode info population failed, %d\n",
					rc);
			goto exit;
		}
		blob = &c_conn->blob_mode_info;
	break;
	default:
		SDE_ERROR_CONN(c_conn, "invalid prop_id: %d\n", prop_id);
		goto exit;
	}

	msm_property_set_blob(&c_conn->property_info,
			blob,
			SDE_KMS_INFO_DATA(info),
			SDE_KMS_INFO_DATALEN(info),
			prop_id);
exit:
	vfree(info);

	return rc;
}

static int _sde_connector_install_properties(struct drm_device *dev,
	struct sde_kms *sde_kms, struct sde_connector *c_conn,
	int connector_type, void *display,
	struct msm_display_info *display_info)
{
	struct dsi_display *dsi_display;
	int rc;
	struct drm_connector *connector;
	u64 panel_id = ~0x0;

	msm_property_install_blob(&c_conn->property_info, "capabilities",
			DRM_MODE_PROP_IMMUTABLE, CONNECTOR_PROP_SDE_INFO);

	rc = sde_connector_set_blob_data(&c_conn->base,
			NULL, CONNECTOR_PROP_SDE_INFO);
	if (rc) {
		SDE_ERROR_CONN(c_conn,
			"failed to setup connector info, rc = %d\n", rc);
		return rc;
	}

	connector = &c_conn->base;

	msm_property_install_blob(&c_conn->property_info, "mode_properties",
			DRM_MODE_PROP_IMMUTABLE, CONNECTOR_PROP_MODE_INFO);

	if (connector_type == DRM_MODE_CONNECTOR_DSI) {
		dsi_display = (struct dsi_display *)(display);
		if (dsi_display && dsi_display->panel &&
			dsi_display->panel->hdr_props.hdr_enabled == true) {
			msm_property_install_blob(&c_conn->property_info,
				"hdr_properties",
				DRM_MODE_PROP_IMMUTABLE,
				CONNECTOR_PROP_HDR_INFO);

			msm_property_set_blob(&c_conn->property_info,
				&c_conn->blob_hdr,
				&dsi_display->panel->hdr_props,
				sizeof(dsi_display->panel->hdr_props),
				CONNECTOR_PROP_HDR_INFO);
		}

		mutex_lock(&c_conn->base.dev->mode_config.mutex);
		sde_connector_fill_modes(&c_conn->base,
						dev->mode_config.max_width,
						dev->mode_config.max_height);
		mutex_unlock(&c_conn->base.dev->mode_config.mutex);
	}

	msm_property_install_volatile_range(
			&c_conn->property_info, "sde_drm_roi_v1", 0x0,
			0, ~0, 0, CONNECTOR_PROP_ROI_V1);

	/* install PP_DITHER properties */
	_sde_connector_install_dither_property(dev, sde_kms, c_conn);

	if (connector_type == DRM_MODE_CONNECTOR_DisplayPort) {
		struct drm_msm_ext_hdr_properties hdr = {0};

		c_conn->hdr_capable = true;

		msm_property_install_blob(&c_conn->property_info,
				"ext_hdr_properties",
				DRM_MODE_PROP_IMMUTABLE,
				CONNECTOR_PROP_EXT_HDR_INFO);

		/* set default values to avoid reading uninitialized data */
		msm_property_set_blob(&c_conn->property_info,
			      &c_conn->blob_ext_hdr,
			      &hdr,
			      sizeof(hdr),
			      CONNECTOR_PROP_EXT_HDR_INFO);

		if (c_conn->ops.install_properties)
			c_conn->ops.install_properties(display, connector);
	}

	msm_property_install_volatile_range(&c_conn->property_info,
		"hdr_metadata", 0x0, 0, ~0, 0, CONNECTOR_PROP_HDR_METADATA);

	msm_property_install_volatile_range(&c_conn->property_info,
		"RETIRE_FENCE", 0x0, 0, ~0, 0, CONNECTOR_PROP_RETIRE_FENCE);

	msm_property_install_range(&c_conn->property_info, "autorefresh",
			0x0, 0, AUTOREFRESH_MAX_FRAME_CNT, 0,
			CONNECTOR_PROP_AUTOREFRESH);

	if (connector_type == DRM_MODE_CONNECTOR_DSI) {
		if (sde_kms->catalog->has_qsync && display_info->qsync_min_fps)
			msm_property_install_enum(&c_conn->property_info,
					"qsync_mode", 0, 0, e_qsync_mode,
					ARRAY_SIZE(e_qsync_mode), 0,
					CONNECTOR_PROP_QSYNC_MODE);

		if (display_info->capabilities & MSM_DISPLAY_CAP_CMD_MODE)
			msm_property_install_enum(&c_conn->property_info,
				"frame_trigger_mode", 0, 0,
				e_frame_trigger_mode,
				ARRAY_SIZE(e_frame_trigger_mode), 0,
				CONNECTOR_PROP_CMD_FRAME_TRIGGER_MODE);

		if (sde_kms->catalog->has_demura) {
			msm_property_install_blob(&c_conn->property_info,
				"DEMURA_PANEL_ID", DRM_MODE_PROP_IMMUTABLE,
				CONNECTOR_PROP_DEMURA_PANEL_ID);
			msm_property_set_blob(&c_conn->property_info,
			      &c_conn->blob_panel_id,
			      &panel_id,
			      sizeof(panel_id),
			      CONNECTOR_PROP_DEMURA_PANEL_ID);
		}
	}

	msm_property_install_range(&c_conn->property_info, "bl_scale",
		0x0, 0, MAX_BL_SCALE_LEVEL, MAX_BL_SCALE_LEVEL,
		CONNECTOR_PROP_BL_SCALE);

	msm_property_install_range(&c_conn->property_info, "sv_bl_scale",
		0x0, 0, MAX_SV_BL_SCALE_LEVEL, MAX_SV_BL_SCALE_LEVEL,
		CONNECTOR_PROP_SV_BL_SCALE);

	c_conn->bl_scale_dirty = false;
	c_conn->bl_scale = MAX_BL_SCALE_LEVEL;
	c_conn->bl_scale_sv = MAX_SV_BL_SCALE_LEVEL;

	if (connector_type == DRM_MODE_CONNECTOR_DisplayPort)
		msm_property_install_range(&c_conn->property_info,
			"supported_colorspaces",
			DRM_MODE_PROP_IMMUTABLE, 0, 0xffff, 0,
			CONNECTOR_PROP_SUPPORTED_COLORSPACES);

	/* enum/bitmask properties */
	msm_property_install_enum(&c_conn->property_info, "topology_name",
			DRM_MODE_PROP_IMMUTABLE, 0, e_topology_name,
			ARRAY_SIZE(e_topology_name), 0,
			CONNECTOR_PROP_TOPOLOGY_NAME);
	msm_property_install_enum(&c_conn->property_info, "topology_control",
			0, 1, e_topology_control,
			ARRAY_SIZE(e_topology_control), 0,
			CONNECTOR_PROP_TOPOLOGY_CONTROL);
	msm_property_install_enum(&c_conn->property_info, "LP",
			0, 0, e_power_mode,
			ARRAY_SIZE(e_power_mode), 0,
			CONNECTOR_PROP_LP);

	return 0;
}

static ssize_t panel_power_state_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct drm_connector *conn;
	struct sde_connector *sde_conn;

	conn = dev_get_drvdata(device);
	sde_conn = to_sde_connector(conn);

	return scnprintf(buf, PAGE_SIZE, "%d\n", sde_conn->last_panel_power_mode);
}

static ssize_t twm_enable_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *conn;
	struct sde_connector *sde_conn;
	struct dsi_display *dsi_display;
	int rc;
	int data;

	conn = dev_get_drvdata(device);
	sde_conn = to_sde_connector(conn);
	dsi_display = (struct dsi_display *) sde_conn->display;
	rc = kstrtoint(buf, 10, &data);
	if (rc) {
		SDE_ERROR("kstrtoint failed, rc = %d\n", rc);
		return -EINVAL;
	}
	sde_conn->twm_en = data ? true : false;
	dsi_display->panel->is_twm_en = sde_conn->twm_en;
	sde_conn->allow_bl_update = data ? false : true;
	SDE_DEBUG("TWM: %s\n", sde_conn->twm_en ? "ENABLED" : "DISABLED");
	return count;
}

static ssize_t twm_enable_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct drm_connector *conn;
	struct sde_connector *sde_conn;

	conn = dev_get_drvdata(device);
	sde_conn = to_sde_connector(conn);

	SDE_DEBUG("TWM: %s\n", sde_conn->twm_en ? "ENABLED" : "DISABLED");
	return scnprintf(buf, PAGE_SIZE, "%d\n", sde_conn->twm_en);
}

static DEVICE_ATTR_RO(panel_power_state);
static DEVICE_ATTR_RW(twm_enable);

static struct attribute *sde_connector_dev_attrs[] = {
	&dev_attr_panel_power_state.attr,
	&dev_attr_twm_enable.attr,
	NULL
};

static const struct attribute_group sde_connector_attr_group = {
	.attrs = sde_connector_dev_attrs,
};
static const struct attribute_group *sde_connector_attr_groups[] = {
	&sde_connector_attr_group,
	NULL,
};

int sde_connector_post_init(struct drm_device *dev, struct drm_connector *conn)
{
	struct sde_connector *c_conn;
	int rc = 0;

	if (!dev || !dev->primary || !dev->primary->kdev || !conn) {
		SDE_ERROR("invalid input param(s)\n");
		rc = -EINVAL;
		return rc;
	}

	c_conn =  to_sde_connector(conn);

	if (conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return rc;

	c_conn->sysfs_dev =
		device_create_with_groups(dev->primary->kdev->class, dev->primary->kdev, 0,
			conn, sde_connector_attr_groups, "sde-conn-%d-%s", conn->index,
			conn->name);
	if (IS_ERR_OR_NULL(c_conn->sysfs_dev)) {
		SDE_ERROR("connector:%d sysfs create failed rc:%ld\n", &c_conn->base.index,
			PTR_ERR(c_conn->sysfs_dev));
		if (!c_conn->sysfs_dev)
			rc = -EINVAL;
		else
			rc = PTR_ERR(c_conn->sysfs_dev);
	}

	return rc;
}

struct drm_connector *sde_connector_init(struct drm_device *dev,
		struct drm_encoder *encoder,
		struct drm_panel *panel,
		void *display,
		const struct sde_connector_ops *ops,
		int connector_poll,
		int connector_type)
{
	struct msm_drm_private *priv;
	struct sde_kms *sde_kms;
	struct sde_connector *c_conn = NULL;
	struct msm_display_info display_info;
	int rc;

	if (!dev || !dev->dev_private || !encoder) {
		SDE_ERROR("invalid argument(s), dev %pK, enc %pK\n",
				dev, encoder);
		return ERR_PTR(-EINVAL);
	}

	priv = dev->dev_private;
	if (!priv->kms) {
		SDE_ERROR("invalid kms reference\n");
		return ERR_PTR(-EINVAL);
	}

	c_conn = kzalloc(sizeof(*c_conn), GFP_KERNEL);
	if (!c_conn) {
		SDE_ERROR("failed to alloc sde connector\n");
		return ERR_PTR(-ENOMEM);
	}

	memset(&display_info, 0, sizeof(display_info));

	rc = drm_connector_init(dev,
			&c_conn->base,
			&sde_connector_ops,
			connector_type);
	if (rc)
		goto error_free_conn;

	spin_lock_init(&c_conn->event_lock);

	c_conn->base.panel = panel;
	c_conn->connector_type = connector_type;
	c_conn->encoder = encoder;
	c_conn->display = display;

	c_conn->dpms_mode = DRM_MODE_DPMS_ON;
	c_conn->lp_mode = 0;
	c_conn->last_panel_power_mode = SDE_MODE_DPMS_ON;
	c_conn->twm_en = false;

	sde_kms = to_sde_kms(priv->kms);
	if (sde_kms->vbif[VBIF_NRT]) {
		c_conn->aspace[SDE_IOMMU_DOMAIN_UNSECURE] =
			sde_kms->aspace[MSM_SMMU_DOMAIN_NRT_UNSECURE];
		c_conn->aspace[SDE_IOMMU_DOMAIN_SECURE] =
			sde_kms->aspace[MSM_SMMU_DOMAIN_NRT_SECURE];
	} else {
		c_conn->aspace[SDE_IOMMU_DOMAIN_UNSECURE] =
			sde_kms->aspace[MSM_SMMU_DOMAIN_UNSECURE];
		c_conn->aspace[SDE_IOMMU_DOMAIN_SECURE] =
			sde_kms->aspace[MSM_SMMU_DOMAIN_SECURE];
	}

	if (ops)
		c_conn->ops = *ops;

	if (ops && ops->atomic_best_encoder && ops->atomic_check)
		c_conn->base.helper_private = &sde_connector_helper_ops_v2;
	else
		c_conn->base.helper_private = &sde_connector_helper_ops;

	c_conn->base.polled = connector_poll;
	c_conn->base.interlace_allowed = 0;
	c_conn->base.doublescan_allowed = 0;

	snprintf(c_conn->name,
			SDE_CONNECTOR_NAME_SIZE,
			"conn%u",
			c_conn->base.base.id);

	c_conn->retire_fence = sde_fence_init(c_conn->name,
			c_conn->base.base.id);
	if (IS_ERR(c_conn->retire_fence)) {
		rc = PTR_ERR(c_conn->retire_fence);
		SDE_ERROR("failed to init fence, %d\n", rc);
		goto error_cleanup_conn;
	}

	mutex_init(&c_conn->lock);

	rc = drm_connector_attach_encoder(&c_conn->base, encoder);
	if (rc) {
		SDE_ERROR("failed to attach encoder to connector, %d\n", rc);
		goto error_cleanup_fence;
	}

	rc = sde_backlight_setup(c_conn, dev);
	if (rc) {
		SDE_ERROR("failed to setup backlight, rc=%d\n", rc);
		goto error_cleanup_fence;
	}

	/* create properties */
	msm_property_init(&c_conn->property_info, &c_conn->base.base, dev,
			priv->conn_property, c_conn->property_data,
			CONNECTOR_PROP_COUNT, CONNECTOR_PROP_BLOBCOUNT,
			sizeof(struct sde_connector_state));

	if (c_conn->ops.post_init) {
		rc = c_conn->ops.post_init(&c_conn->base, display);
		if (rc) {
			SDE_ERROR("post-init failed, %d\n", rc);
			goto error_cleanup_fence;
		}
	}

	rc = sde_connector_install_panel_params(c_conn);
	if (rc) {
		SDE_ERROR_CONN(c_conn,
			"failed to install property for panel params. rc =%d\n",
							rc);
			goto error_cleanup_fence;
	}

	rc = sde_connector_get_info(&c_conn->base, &display_info);
	if (!rc && (connector_type == DRM_MODE_CONNECTOR_DSI) &&
			(display_info.capabilities & MSM_DISPLAY_CAP_VID_MODE))
		sde_connector_register_event(&c_conn->base,
			SDE_CONN_EVENT_VID_FIFO_OVERFLOW,
			sde_connector_handle_disp_recovery,
			c_conn);

	rc = _sde_connector_install_properties(dev, sde_kms, c_conn,
		connector_type, display, &display_info);
	if (rc)
		goto error_cleanup_fence;

	if (connector_type == DRM_MODE_CONNECTOR_DSI &&
			sde_kms->catalog->has_demura) {
		rc = sde_connector_register_event(&c_conn->base,
			SDE_CONN_EVENT_PANEL_ID,
			sde_connector_handle_panel_id, c_conn);
		if (rc)
			SDE_ERROR("register panel id event err %d\n", rc);
	}

	rc = msm_property_install_get_status(&c_conn->property_info);
	if (rc) {
		SDE_ERROR("failed to create one or more properties\n");
		goto error_destroy_property;
	}

	_sde_connector_lm_preference(c_conn, sde_kms,
			display_info.display_type);

	sde_hw_ctl_set_preference(sde_kms->catalog,
			  display_info.display_type);

	SDE_DEBUG("connector %d attach encoder %d\n",
			c_conn->base.base.id, encoder->base.id);

	INIT_DELAYED_WORK(&c_conn->status_work,
			sde_connector_check_status_work);

	return &c_conn->base;

error_destroy_property:
	if (c_conn->blob_caps)
		drm_property_blob_put(c_conn->blob_caps);
	if (c_conn->blob_hdr)
		drm_property_blob_put(c_conn->blob_hdr);
	if (c_conn->blob_dither)
		drm_property_blob_put(c_conn->blob_dither);
	if (c_conn->blob_mode_info)
		drm_property_blob_put(c_conn->blob_mode_info);
	if (c_conn->blob_ext_hdr)
		drm_property_blob_put(c_conn->blob_ext_hdr);

	msm_property_destroy(&c_conn->property_info);
error_cleanup_fence:
	mutex_destroy(&c_conn->lock);
	sde_fence_deinit(c_conn->retire_fence);
error_cleanup_conn:
	drm_connector_cleanup(&c_conn->base);
error_free_conn:
	kfree(c_conn);

	return ERR_PTR(rc);
}

static int _sde_conn_enable_hw_recovery(struct drm_connector *connector)
{
	struct sde_connector *c_conn;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return -EINVAL;
	}
	c_conn = to_sde_connector(connector);

	if (c_conn->encoder)
		sde_encoder_enable_recovery_event(c_conn->encoder);

	return 0;
}

int sde_connector_register_custom_event(struct sde_kms *kms,
		struct drm_connector *conn_drm, u32 event, bool val)
{
	int ret = -EINVAL;

	switch (event) {
	case DRM_EVENT_SYS_BACKLIGHT:
		ret = 0;
		break;
	case DRM_EVENT_PANEL_DEAD:
		ret = 0;
		break;
	case DRM_EVENT_SDE_HW_RECOVERY:
		ret = _sde_conn_enable_hw_recovery(conn_drm);
		break;
	default:
		break;
	}
	return ret;
}

int sde_connector_event_notify(struct drm_connector *connector, uint32_t type,
		uint32_t len, uint32_t val)
{
	struct drm_event event;
	int ret;

	if (!connector) {
		SDE_ERROR("invalid connector\n");
		return -EINVAL;
	}

	switch (type) {
	case DRM_EVENT_SYS_BACKLIGHT:
	case DRM_EVENT_PANEL_DEAD:
	case DRM_EVENT_SDE_HW_RECOVERY:
		ret = 0;
		break;
	default:
		SDE_ERROR("connector %d, Unsupported event %d\n",
				connector->base.id, type);
		return -EINVAL;
	}

	event.type = type;
	event.length = len;
	msm_mode_object_event_notify(&connector->base, connector->dev, &event,
			(u8 *)&val);

	SDE_EVT32(connector->base.id, type, len, val);
	SDE_DEBUG("connector:%d hw recovery event(%d) value (%d) notified\n",
			connector->base.id, type, val);

	return ret;
}
