/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEADER_SIZE 200//this is just a big enough number, to accomodate all the elements, will have to calculate the exact value later
#define FOOTER_SIZE 2 //this is not really needed but just added it for uniformity
#define CCD_PIXEL_BUFFER_SIZE (6000) // Adjust as needed for header, data, and footer

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

// Global variables for header values, these are all from the image sent
char VersionNo[4] = "DCAM";
int MajorVersion = 1;
int FileFormatNo = 0;
int CustomizationNo = 1;
int CameraType = 1;  // Assuming 1: Line scanner camera
int DataInfo[5] = {0};  // {SizeX, SizeY, DataSet, DataByte}
int MeasureMode = 1;  // Assuming 1: Line Scanning Mode
int SheetData = 0;  // Assuming 0: Image data
int Profile = 1;  // Assuming 1: Show
char ScaleUnitChars[20] = "ADU";  // Default scale unit characters

//Test Buffer
uint16_t TestBuffer1[300] = {2387, 2151, 2067, 1324, 1056, 2498, 2401, 1869, 1786, 2388, 1785, 1758, 1449, 1496, 1499, 1029, 1707, 2099, 1147, 2440, 1925, 2357, 2081, 1354, 2088, 2023, 2342, 1083, 1418, 2465, 2103, 1068, 1324, 1242, 2147, 1934, 2451, 2318, 1143, 1005, 1295, 1554, 2060, 2497, 2244, 1067, 2118, 2070, 1558, 2298, 1332, 1405, 1235, 1349, 1295, 1643, 1274, 1005, 1617, 1329, 2177, 2066, 2150, 1655, 1459, 2080, 1860, 1848, 1527, 1496, 1626, 2251, 1762, 1955, 1166, 1831, 1066, 1192, 1966, 2180, 1322, 2465, 1515, 2353, 1626, 1934, 1986, 2034, 1030, 1749, 1625, 1993, 2296, 2243, 1363, 2259, 1000, 1351, 1276, 1411, 1233, 1119, 1795, 1919, 2191, 2232, 1575, 2364, 1569, 2058, 1805, 1583, 1703, 1476, 1979, 2076, 1211, 1425, 1688, 2329, 1343, 1656, 1890, 1675, 1692, 1568, 2234, 1121, 2415, 2177, 2205, 1808, 1475, 1223, 1575, 1713, 1372, 2247, 1565, 1867, 1665, 1860, 1675, 1447, 1927, 1041, 2211, 1335, 2287,
        1266, 1964, 2299, 1576, 1837, 2377, 1567, 1722, 1177, 1089, 1948, 2019, 1465, 2242, 1288, 2287, 2412, 1054, 1896, 1331, 1482, 1338, 1890, 1352, 1528, 1058, 1597, 1201, 2221, 1063, 1727, 1239, 1011, 1320, 1545, 1965, 1564, 2266, 1553, 2382, 2294, 1555, 1370, 1211, 1201, 1369, 1228, 1460, 1488, 2106, 1176, 1861, 1387, 1799, 1847, 1695, 2176, 2258, 1413, 2488, 2063, 1160, 1328, 2266, 2257, 1286, 2402, 1995, 1398, 1353, 1351, 1971, 1646, 2457, 1690, 2054, 2464, 1487, 1223, 2314, 2303, 1198, 2299, 2032, 2094, 1295, 2190, 2080, 2282, 2174, 2472, 2403, 2271, 1594, 1379, 1875, 1847, 1980, 1235, 1980, 1310, 1520, 2159, 2175, 1145, 1534, 1654, 1102, 2063, 2055, 1519, 1852, 1417, 1162, 2147, 1608, 2342, 1712, 1543, 1615, 1818, 1048, 2462, 1684, 2022, 1820, 1470, 1868, 1762, 2257, 1826, 1683, 1630, 1730, 1028, 1178, 1507, 1041, 1847, 1439, 1036, 2477, 2276, 1683, 2144, 2043, 1237, 1445, 2192, 2348, 1750};

uint16_t TestBuffer2[300] = {1097, 1785, 1266, 2192, 1686, 1095, 2440, 2194, 2311, 2031, 1371, 1921, 1276, 2123, 2225, 2135, 2284, 1833, 1943, 2343, 2426, 1091, 2320, 1119, 1763, 1409, 1062, 1258, 1448, 1732, 1134, 1139, 1883, 1719, 1229, 2039, 1448, 2227, 2240, 1490, 1965, 1029, 1459, 1956, 1986, 2250, 2317, 2095, 2401, 1315, 2311, 1793, 2373, 2114, 2039, 2455, 1673, 2339, 1271, 1235, 1217, 1766, 1046, 1968, 2286, 1819, 2268, 1734, 2378, 2194, 2360, 2114, 1907, 2055, 1680, 2089, 2400, 2058, 1278, 2181, 1416, 2353, 1049, 1811, 2151, 1267, 1707, 2036, 2412, 2385, 2172, 1266, 2024, 2444, 2144, 1900, 1085, 2290, 2154, 1154, 1402, 1916, 1597, 1675, 2390, 1231, 1384, 1607, 1022, 1308, 2153, 1852, 2429, 1974, 2072, 1599, 1521, 1919, 2436, 1956, 1008, 1356, 1036, 1435, 2225, 1914, 1602, 2318, 2195, 1950, 1639, 2172, 2208, 1161, 1731, 1765, 1636, 2379, 2355, 1491, 1405, 1974, 1335, 1907, 1997, 1773, 2293, 2374, 1668,
    2223, 1320, 2134, 1289, 1723, 2411, 2175, 1856, 1737, 1675, 1141, 1547, 1516, 1924, 1377, 1462, 1914, 1823, 2128, 1510, 1388, 2329, 1166, 1947, 1333, 2177, 1757, 1336, 1571, 1412, 1351, 2159, 1284, 1353, 2141, 2351, 2103, 1243, 1413, 1832, 1592, 2308, 1912, 2215, 1216, 1773, 2012, 2169, 2187, 1520, 1927, 1154, 2389, 1161, 2096, 1224, 2330, 1023, 1348, 2133, 1011, 1805, 1068, 1675, 2283, 1060, 1394, 1537, 1397, 2291, 1094, 1397, 2413, 1171, 1461, 2170, 1493, 1532, 1209, 1506, 1132, 1017, 1717, 1227, 1333, 1498, 1446, 1289, 1089, 1323, 1065, 2279, 2086, 1969, 2211, 1599, 1471, 2132, 2165, 1764, 1554, 1475, 2399, 2317, 1063, 2206, 1686, 2155, 1985, 1574, 2099, 1632, 1071, 1202, 1986, 1481, 1694, 1199, 1407, 2302, 2036, 1097, 1780, 2393, 2150, 1072, 1277, 1464, 1837, 1467, 1215, 1052, 2351, 1615, 1663, 1993, 1939, 2237, 1240, 1782, 1644, 2167, 1617, 1099, 1056, 1858, 2180, 1445, 1155, 2036, 2215};

uint16_t TestBuffer3[300] = {2319, 2225, 1960, 1253, 1731, 1797, 2408, 1608, 1322, 1301, 1230, 2370, 1410, 2095, 2243, 2071, 1511, 2274, 2269, 1112, 1138, 1359, 1511, 2291, 1618, 1373, 1809, 1305, 1957, 1716, 1353, 1012, 1014, 2166, 1882, 1236, 2010, 2205, 2282, 2155, 1255, 2315, 1444, 1292, 2327, 1180, 2494, 2175, 2396, 2069, 1626, 1829, 2454, 1805, 1030, 1933, 1365, 2087, 1502, 2388, 1928, 2095, 1769, 1976, 1679, 2000, 1779, 1749, 2348, 2251, 2402, 1151, 1891, 1163, 1327, 2223, 2060, 1248, 2336, 1006, 1005, 2315, 2358, 1001, 2433, 1620, 2133, 1726, 2093, 2097, 1280, 1241, 2218, 2379, 1784, 1892, 1320, 1100, 1968, 2353, 1185, 1272, 1243, 2365, 1750, 2058, 1310, 1329, 1415, 2176, 2397, 2453, 2099, 2245, 2217, 1361, 2047, 1091, 2175, 1005, 1437, 1317, 1652, 2157, 1422, 2087, 2316, 2426, 2130, 2220, 1808, 1618, 1171, 2158, 1385, 1850, 1628, 1159, 1574, 2455, 1475, 2303, 1425, 1395, 1724, 1440, 1999, 2001, 1408,
        1097, 1261, 2461, 2110, 1285, 2449, 2119, 2476, 2233, 1076, 1909, 1628, 1829, 1109, 1390, 1858, 1646, 1448, 2421, 2436, 2069, 2365, 2414, 1816, 1591, 1046, 1090, 2184, 2393, 2343, 2358, 1436, 1280, 1859, 1701, 1737, 1194, 1147, 1003, 2339, 2305, 1665, 1759, 1918, 1008, 1052, 2097, 2329, 1061, 1444, 1571, 1392, 1166, 1235, 1907, 1866, 2174, 1516, 1622, 1991, 2165, 1403, 2003, 1686, 2335, 2187, 1921, 2263, 2111, 1641, 2042, 2500, 2006, 1538, 2399, 2060, 2244, 1418, 1007, 2121, 2037, 1716, 1103, 2445, 2192, 1377, 1282, 1799, 1584, 1196, 2006, 1109, 2314, 1553, 1245, 1209, 1548, 1660, 2453, 1617, 1366, 1469, 1490, 2106, 2019, 2467, 1702, 1660, 1899, 1537, 2114, 1048, 1184, 1916, 1599, 1187, 1486, 2473, 2202, 1571, 1949, 1792, 1229, 1455, 1989, 1616, 1137, 2339, 2328, 1383, 1305, 2260, 2472, 2088, 2036, 1938, 1204, 1997, 1056, 2297, 1080, 1037, 1835, 2242, 2089, 1131, 2135, 1011, 1803, 1566, 1650};
// Buffers
char header[HEADER_SIZE];
char footer[FOOTER_SIZE];


volatile uint16_t CCDPixelBuffer[CCD_PIXEL_BUFFER_SIZE];
volatile uint16_t headertest[200];
//this is the variable to initiate the dma
volatile uint8_t start_command_received = 0;
volatile uint8_t continuous_enabled = 0;
int experiment = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void encodeData(uint16_t* data_buffer, uint32_t data_size);
void InitializeHeaderFooter();
void CDCReceiveCallback(uint8_t* Buf, uint32_t Len);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void Configure_SH_Signal(uint32_t period, uint32_t pulse);
void Configure_ICG_Signal(uint32_t period, uint32_t pulse);
void Configure_MasterClock_Signal(uint32_t period, uint32_t pulse);
void CalculateAndSetIntegrationTime(uint32_t integration_time_us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//all the main implementations are after line 547 and in the main function

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  InitializeHeaderFooter();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //ICG
//  __HAL_TIM_SET_COUNTER(&htim2, 66);// 600ns delay
////  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //fM
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //ADC
//  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); //SH

  if (!start_command_received) {
  CDC_Transmit_FS("Target Ready\r\n",14);
  HAL_Delay(1000);}

//  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 525000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8820-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 168-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 84-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 168-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 42-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 525000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8347-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */





//when the stm receives anything, it checks if it is the string start

void CDCReceiveCallback(uint8_t* Buf, uint32_t Len) {
    // Ensure the buffer is null-terminated
    Buf[Len] = '\0';

    // Check for the "start" command
    if (strncmp((char*)Buf, "start", 5) == 0) {
    	CDC_Transmit_FS((uint8_t*)TestBuffer, 600);
    }
}

void CalculateAndSetIntegrationTime(uint32_t integration_time_us) {
    // Calculate the SH period (T_SH) in timer ticks
    uint32_t T_SH_ticks = (84 * integration_time_us);  // Convert microseconds to timer ticks

    // Minimum SH pulse width is 1 µs, which translates to 84 timer ticks at 84 MHz
    uint32_t SH_pulse_ticks = 84;

    // Configure the SH signal with the calculated period and pulse width
    Configure_SH_Signal(T_SH_ticks, SH_pulse_ticks);
}


void InitializeHeaderFooter(void) {
    // Construct the header
    snprintf(header, HEADER_SIZE,
        "Version: %s\nMajor: %d\nFileFormat: %d\nCustomization: %d\nCameraType: %d\n"
        "SizeX: %d\nSizeY: %d\nDataSet: %d\nDataByte: %d\nMeasureMode: %d\nSheetData: %d\n"
        "Profile: %d\nScaleUnit: %s\n",
        VersionNo, MajorVersion, FileFormatNo, CustomizationNo, CameraType,
        DataInfo[0], DataInfo[1], DataInfo[2], DataInfo[3],
        MeasureMode, SheetData, Profile, ScaleUnitChars);

    // Construct the footer (this example just uses a simple footer, customize as needed)
    snprintf(footer, FOOTER_SIZE, "\n");
}

// Function to shift data and add header and footer
void encodeData(uint16_t* data_buffer, uint32_t data_size) {

	// Copy the header to the start of the buffer
	    memcpy((void*)CCDPixelBuffer, (const void*)header, HEADER_SIZE);

	    // Copy the footer to the end of the data
	    memcpy((void*)&CCDPixelBuffer[CCD_PIXEL_BUFFER_SIZE - FOOTER_SIZE], (const void*)footer, FOOTER_SIZE);
}

void Configure_SH_Signal(uint32_t period, uint32_t pulse) {
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim5.Init.Prescaler = 1-1;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = period-1;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        // Configuration Error
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //ICG
    __HAL_TIM_SET_COUNTER(&htim2, 66);// 600ns delay
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //fM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); //SH
}

void Configure_ICG_Signal(uint32_t period, uint32_t pulse) {
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Init.Prescaler = 1-1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = period-1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        // Configuration Error
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //ICG
    __HAL_TIM_SET_COUNTER(&htim2, 66);// 600ns delay
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //fM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); //SH
}

void Configure_MasterClock_Signal(uint32_t period, uint32_t pulse) {
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Init.Prescaler = 1-1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = period-1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        // Configuration Error
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //ICG
    __HAL_TIM_SET_COUNTER(&htim2, 66);// 600ns delay
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //fM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); //SH
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
