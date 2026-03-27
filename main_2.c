/**
  ******************************************************************************
  * @file    JPEG/JPEG_DecodingUsingFs_DMA/CM7/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use the HW JPEG to Decode a JPEG file with DMA method.
  *          This is the main program for Cortex-M7
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "decode_dma.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/** @addtogroup STM32H7xx_HAL_Examples
  * @{
  */

/** @addtogroup JPEG_DecodingUsingFs_DMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define MAZE_WIDTH  80
#define MAZE_HEIGHT 80
static uint8_t mazeArray[MAZE_HEIGHT][MAZE_WIDTH];
#define QUEUE_SIZE 1600
static uint16_t queue[QUEUE_SIZE];
static uint16_t queueHead = 0;
static uint16_t queueTail = 0;

static uint8_t visited[MAZE_HEIGHT][MAZE_WIDTH / 8 + 1];
static uint16_t prev[MAZE_HEIGHT][MAZE_WIDTH];
static const int8_t dx[] = {0, 1, 0, -1};
static const int8_t dy[] = {-1, 0, 1, 0};

#define MAX_FILES 20
#define MAX_FILENAME_LEN 256

static char fileList[MAX_FILES][MAX_FILENAME_LEN];
static uint8_t fileCount = 0;
static uint8_t selectedFile = 0;
static uint8_t menuActive = 1;

FATFS SDFatFs;  /* File system object for SD card logical drive */
char SDPath[4]; /* SD card logical drive path */
FIL JPEG_File;  /* File object */

uint32_t JpegProcessing_End = 0;
static uint32_t LCD_X_Size = 0;
static uint32_t LCD_Y_Size = 0;

JPEG_HandleTypeDef    JPEG_Handle;

static DMA2D_HandleTypeDef    DMA2D_Handle;
static JPEG_ConfTypeDef       JPEG_Info;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void LCD_BriefDisplay(void);
static void LCD_FileErrorDisplay(void);
static void DMA2D_CopyBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize, uint32_t ChromaSampling);
static void SD_Initialize(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);

static void ScanJpegFiles(void);
static void DisplayFileMenu(void);
static void OpenAndDisplayJpeg(uint8_t fileIndex);
static void ExtractMazeArray(uint32_t xPos, uint32_t yPos, JPEG_ConfTypeDef *jpegInfo);
static void solveMaze(uint32_t xPos, uint32_t yPos, JPEG_ConfTypeDef *jpegInfo);
static void drawPathOnOriginal(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY,
                               uint32_t xPos, uint32_t yPos, JPEG_ConfTypeDef *jpegInfo);
static void findPathBFS(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY);
static void clearVisited(void);
static void enqueue(uint16_t x, uint16_t y);
static uint8_t dequeue(uint16_t *x, uint16_t *y);
static int8_t isQueueEmpty(void);
static void setVisited(uint16_t y, uint16_t x);
static uint8_t isVisited(uint16_t y, uint16_t x);
static uint8_t ReadJoystick(void);
static void MX_JOYSTICK_GPIO_Init(void);
static void ResetMazeState(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

#ifndef JPEG_OUTPUT_BUFFER_SIZE
#define JPEG_OUTPUT_BUFFER_SIZE (1024 * 1024)
#endif

static void HardResetJPEG(void)
{
    __HAL_RCC_JPEG_FORCE_RESET();
    HAL_Delay(50);
    __HAL_RCC_JPEG_RELEASE_RESET();
    HAL_Delay(50);

    for(int i = 0; i < 100; i++)
    {
        asm("nop");
    }
}

static void ResetMazeState(void)
{
    uint16_t y, x;

    if (HAL_DMA2D_GetState(&DMA2D_Handle) == HAL_DMA2D_STATE_BUSY)
    {
        HAL_DMA2D_Abort(&DMA2D_Handle);
        HAL_Delay(50);
    }
    HAL_DMA2D_DeInit(&DMA2D_Handle);
    memset(&DMA2D_Handle, 0, sizeof(DMA2D_Handle));
    JPEG_ResetDecoderState();
    HardResetJPEG();

    memset(&JPEG_Handle, 0, sizeof(JPEG_Handle));
    JPEG_Handle.Instance = JPEG;
    HAL_JPEG_Init(&JPEG_Handle);

    for(y = 0; y < MAZE_HEIGHT; y++)
    {
        memset(mazeArray[y], 0, MAZE_WIDTH);
        memset(visited[y], 0, MAZE_WIDTH / 8 + 1);
    }

    for(y = 0; y < MAZE_HEIGHT; y++)
    {
        for(x = 0; x < MAZE_WIDTH; x++)
        {
            prev[y][x] = 0xFFFF;
        }
    }

    queueHead = 0;
    queueTail = 0;
    memset(queue, 0, QUEUE_SIZE * sizeof(uint16_t));

    menuActive = 1;
    JpegProcessing_End = 0;

    uint32_t *jpeg_buf = (uint32_t *)JPEG_OUTPUT_DATA_BUFFER;
    for(uint32_t i = 0; i < JPEG_OUTPUT_BUFFER_SIZE / 4; i++)
    {
        jpeg_buf[i] = 0;
    }

    uint32_t *fb = (uint32_t *)LCD_FRAME_BUFFER;
    for(uint32_t i = 0; i < LCD_X_Size * LCD_Y_Size; i++)
    {
        fb[i] = 0xFFFFFFFF;
    }

    f_close(&JPEG_File);
    memset(&JPEG_File, 0, sizeof(JPEG_File));

    SCB_CleanInvalidateDCache();
    HAL_Delay(100);
}

static void OpenAndDisplayJpeg(uint8_t fileIndex)
{
    uint32_t xPos = 0, yPos = 0;
    char filepath[MAX_FILENAME_LEN];
    uint32_t *fb = (uint32_t *)LCD_FRAME_BUFFER;
    uint32_t timeout;
    FRESULT res;

    JPEG_ResetDecoderState();

    sprintf(filepath, "%s/%s", SDPath, fileList[fileIndex]);
    menuActive = 0;

    f_close(&JPEG_File);

    UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
    UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLUE);
    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
    UTIL_LCD_DisplayStringAt(0, LINE(2), (uint8_t *)"Loading...", CENTER_MODE);

    SCB_InvalidateDCache();

    uint32_t *jpeg_buf = (uint32_t *)JPEG_OUTPUT_DATA_BUFFER;
    for(uint32_t i = 0; i < JPEG_OUTPUT_BUFFER_SIZE / 4; i++)
    {
        jpeg_buf[i] = 0;
    }
    SCB_CleanInvalidateDCache();

    res = f_open(&JPEG_File, filepath, FA_READ);
    if (res == FR_OK)
    {
        JpegProcessing_End = 0;

        f_lseek(&JPEG_File, 0);

        HAL_JPEG_DeInit(&JPEG_Handle);
        HAL_Delay(50);
        JPEG_Handle.Instance = JPEG;
        HAL_JPEG_Init(&JPEG_Handle);

        if (JPEG_Decode_DMA(&JPEG_Handle, &JPEG_File, JPEG_OUTPUT_DATA_BUFFER) != HAL_OK)
        {
            UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
            UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_RED);
            UTIL_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)"Decode Init Error!", CENTER_MODE);
            HAL_Delay(2000);
            menuActive = 1;
            ResetMazeState();
            DisplayFileMenu();
            return;
        }

        timeout = 0;
        do
        {
            JpegProcessing_End = JPEG_InputHandler(&JPEG_Handle);
            SCB_InvalidateDCache();
            HAL_Delay(10);
            timeout++;
            if (timeout > 500)
            {
                break;
            }
        } while(JpegProcessing_End == 0);

        if (JpegProcessing_End == 2)
        {
            UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
            UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_RED);
            UTIL_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)"Decode Error!", CENTER_MODE);
            HAL_Delay(2000);
            menuActive = 1;
            ResetMazeState();
            DisplayFileMenu();
            return;
        }

        HAL_JPEG_GetInfo(&JPEG_Handle, &JPEG_Info);

        SCB_InvalidateDCache_by_Addr((uint32_t*)JPEG_OUTPUT_DATA_BUFFER, JPEG_OUTPUT_BUFFER_SIZE);

        xPos = (LCD_X_Size - JPEG_Info.ImageWidth) / 2;
        yPos = (LCD_Y_Size - JPEG_Info.ImageHeight) / 2;

        for(uint32_t i = 0; i < LCD_X_Size * LCD_Y_Size; i++)
        {
            fb[i] = 0xFFFFFFFF;
        }
        SCB_CleanInvalidateDCache();

        if (HAL_DMA2D_GetState(&DMA2D_Handle) == HAL_DMA2D_STATE_BUSY)
        {
            HAL_DMA2D_Abort(&DMA2D_Handle);
            HAL_Delay(10);
        }
        HAL_DMA2D_DeInit(&DMA2D_Handle);
        memset(&DMA2D_Handle, 0, sizeof(DMA2D_Handle));

        DMA2D_CopyBuffer((uint32_t *)JPEG_OUTPUT_DATA_BUFFER, (uint32_t *)LCD_FRAME_BUFFER,
                        xPos, yPos, JPEG_Info.ImageWidth, JPEG_Info.ImageHeight,
                        JPEG_Info.ChromaSubsampling);

        HAL_DMA2D_PollForTransfer(&DMA2D_Handle, 1000);

        SCB_InvalidateDCache();
        ExtractMazeArray(xPos, yPos, &JPEG_Info);
        solveMaze(xPos, yPos, &JPEG_Info);

        f_close(&JPEG_File);

        UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLACK);
        UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
        UTIL_LCD_SetFont(&Font16);
        char msg[] = "LEFT: Back to menu";
        UTIL_LCD_DisplayStringAt(10, LCD_Y_Size - 30, (uint8_t *)msg, LEFT_MODE);

        SCB_CleanInvalidateDCache();
    }
    else
    {
        UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
        UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_RED);
        UTIL_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)"File Open Error!", CENTER_MODE);
        HAL_Delay(2000);
        menuActive = 1;
        ResetMazeState();
        DisplayFileMenu();
    }
}

static void ScanJpegFiles(void)
{
    FRESULT res;
    DIR dir;
    FILINFO fno;

    fileCount = 0;
    res = f_opendir(&dir, SDPath);

    if (res == FR_OK) {
        while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0 && fileCount < MAX_FILES) {
            char *ext = strrchr(fno.fname, '.');
            if (ext && (strcmp(ext, ".jpg") == 0 || strcmp(ext, ".JPG") == 0)) {
                strcpy(fileList[fileCount], fno.fname);
                fileCount++;
            }
        }
        f_closedir(&dir);
    }
}

static void DisplayFileMenu(void)
{
    uint32_t *fb = (uint32_t *)LCD_FRAME_BUFFER;
    for(uint32_t i = 0; i < LCD_X_Size * LCD_Y_Size; i++) {
        fb[i] = 0xFFFFFFFF;
    }
    SCB_CleanDCache();

    UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
    UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLUE);
    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
    UTIL_LCD_SetFont(&Font16);

    UTIL_LCD_FillRect(0, 0, LCD_X_Size, 40, UTIL_LCD_COLOR_BLUE);
    UTIL_LCD_DisplayStringAt(0, 10, (uint8_t *)"JPEG Files on SD Card", CENTER_MODE);

    UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_WHITE);
    UTIL_LCD_SetFont(&Font16);

    for (uint8_t i = 0; i < fileCount; i++) {
        uint16_t yPos = 50 + i * 25;

        if (i == selectedFile) {
            UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLUE);
            UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
            UTIL_LCD_FillRect(20, yPos - 2, LCD_X_Size - 40, 22, UTIL_LCD_COLOR_BLUE);
        } else {
            UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_WHITE);
            UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_BLACK);
        }

        UTIL_LCD_DisplayStringAt(45, yPos, (uint8_t *)fileList[i], LEFT_MODE);
    }

    UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_LIGHTGRAY);
    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_BLACK);
    UTIL_LCD_FillRect(0, LCD_Y_Size - 30, LCD_X_Size, 30, UTIL_LCD_COLOR_LIGHTGRAY);

    char instructions[] = "UP/DOWN: Select   RIGHT: Open   LEFT: Back to menu";
    UTIL_LCD_DisplayStringAt(0, LCD_Y_Size - 25, (uint8_t *)instructions, CENTER_MODE);

    SCB_CleanDCache();
}

static void ExtractMazeArray(uint32_t xPos, uint32_t yPos, JPEG_ConfTypeDef *jpegInfo)
{
    uint32_t *pFrameBuffer = (uint32_t *)LCD_FRAME_BUFFER;
    uint32_t pixelColor;
    uint8_t r, g, b;
    uint16_t y, x;
    uint8_t scale = 6;
    for(y = 0; y < MAZE_HEIGHT; y++) {
        for(x = 0; x < MAZE_WIDTH; x++) {
            mazeArray[y][x] = 0;
        }
    }

    for(y = 0; y < MAZE_HEIGHT; y++)
    {
        for(x = 0; x < MAZE_WIDTH; x++)
        {
            uint32_t sumBrightness = 0;
            uint16_t samples = 0;

            for(uint16_t dy = 0; dy < scale; dy++)
            {
                for(uint16_t dx = 0; dx < scale; dx++)
                {
                    uint16_t srcY = y * scale + dy;
                    uint16_t srcX = x * scale + dx;

                    if(srcY < jpegInfo->ImageHeight && srcX < jpegInfo->ImageWidth)
                    {
                        pixelColor = pFrameBuffer[(yPos + srcY) * LCD_X_Size + (xPos + srcX)];

                        r = (pixelColor >> 16) & 0xFF;
                        g = (pixelColor >> 8) & 0xFF;
                        b = pixelColor & 0xFF;

                        sumBrightness += (r + g + b) / 3;
                        samples++;
                    }
                }
            }

            if(samples > 0) {
                uint8_t avgBrightness = sumBrightness / samples;
                mazeArray[y][x] = (avgBrightness > 128) ? 1 : 0;
            } else {
                mazeArray[y][x] = 0;
            }
        }
    }
}

static void DisplayMazeFromArray(void)
{
    uint32_t *pFrameBuffer = (uint32_t *)LCD_FRAME_BUFFER;
    uint16_t y, x;
    uint8_t scale = 6;

    UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);

    uint16_t displayWidth = MAZE_WIDTH * scale;
    uint16_t displayHeight = MAZE_HEIGHT * scale;
    uint16_t displayXPos = (LCD_X_Size - displayWidth) / 2;
    uint16_t displayYPos = (LCD_Y_Size - displayHeight) / 2;

    for(y = 0; y < MAZE_HEIGHT; y++)
    {
        for(x = 0; x < MAZE_WIDTH; x++)
        {
            uint32_t color = (mazeArray[y][x] == 1) ? 0xFFFFFFFF : 0xFF000000;

            for(uint16_t dy = 0; dy < scale; dy++)
            {
                for(uint16_t dx = 0; dx < scale; dx++)
                {
                    pFrameBuffer[(displayYPos + y * scale + dy) * LCD_X_Size +
                                 (displayXPos + x * scale + dx)] = color;
                }
            }
        }
    }

    SCB_CleanDCache();
}
static void setVisited(uint16_t y, uint16_t x)
{
    visited[y][x / 8] |= (1 << (x % 8));
}

static uint8_t isVisited(uint16_t y, uint16_t x)
{
    return (visited[y][x / 8] >> (x % 8)) & 1;
}

static void clearVisited(void)
{
    for(uint16_t y = 0; y < MAZE_HEIGHT; y++)
    {
        for(uint16_t x = 0; x < MAZE_WIDTH / 8 + 1; x++)
        {
            visited[y][x] = 0;
        }
    }
}

static void enqueue(uint16_t x, uint16_t y)
{
    uint16_t next = (queueTail + 1) % QUEUE_SIZE;
    if(next != queueHead)
    {
        queue[queueTail] = (y << 8) | x;
        queueTail = next;
    }
}

static uint8_t dequeue(uint16_t *x, uint16_t *y)
{
    if(queueHead == queueTail) return 0;

    uint16_t packed = queue[queueHead];
    *x = packed & 0xFF;
    *y = (packed >> 8) & 0xFF;
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    return 1;
}

static int8_t isQueueEmpty(void)
{
    return queueHead == queueTail;
}

static void findPathBFS(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY)
{
    uint16_t y, x;

    clearVisited();

    for(y = 0; y < MAZE_HEIGHT; y++)
    {
        for(x = 0; x < MAZE_WIDTH; x++)
        {
            prev[y][x] = 0xFFFF;
        }
    }

    queueHead = 0;
    queueTail = 0;

    enqueue(startX, startY);
    setVisited(startY, startX);
    prev[startY][startX] = (startY << 8) | startX;

    while(!isQueueEmpty())
    {
        uint16_t currentX, currentY;
        if(!dequeue(&currentX, &currentY)) break;

        if(currentX == endX && currentY == endY)
        {
            return;
        }

        for(int dir = 0; dir < 4; dir++)
        {
            uint16_t newX = currentX + dx[dir];
            uint16_t newY = currentY + dy[dir];

            if(newX < MAZE_WIDTH && newY < MAZE_HEIGHT)
            {
                if(mazeArray[newY][newX] == 1 && !isVisited(newY, newX))
                {
                    setVisited(newY, newX);
                    prev[newY][newX] = (currentY << 8) | currentX;
                    enqueue(newX, newY);
                }
            }
        }
    }
}
static void drawPathOnOriginal(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY,
                               uint32_t xPos, uint32_t yPos, JPEG_ConfTypeDef *jpegInfo)
{
    uint32_t *fb = (uint32_t *)LCD_FRAME_BUFFER;
    uint8_t scale = 6;

    uint16_t pathX[MAZE_WIDTH * MAZE_HEIGHT];
    uint16_t pathY[MAZE_WIDTH * MAZE_HEIGHT];
    uint16_t len = 0;

    uint16_t x = endX, y = endY;
    while (x != startX || y != startY) {
        pathX[len] = x;
        pathY[len] = y;
        len++;

        uint16_t p = prev[y][x];
        if (p == 0xFFFF) break;
        x = p & 0xFF;
        y = (p >> 8) & 0xFF;
    }
    pathX[len] = startX;
    pathY[len] = startY;
    len++;
    for (uint16_t i = 0; i < len - 1; i++) {
        uint16_t x1 = xPos + (pathX[i] * scale + scale/2);
        uint16_t y1 = yPos + (pathY[i] * scale + scale/2);
        uint16_t x2 = xPos + (pathX[i+1] * scale + scale/2);
        uint16_t y2 = yPos + (pathY[i+1] * scale + scale/2);

        int16_t dx = abs(x2 - x1), dy = abs(y2 - y1);
        int16_t sx = x1 < x2 ? 1 : -1, sy = y1 < y2 ? 1 : -1;
        int16_t err = dx - dy;

        uint16_t x = x1, y = y1;
        while (1) {
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int32_t px = x + dx, py = y + dy;
                    if (px >= 0 && px < LCD_X_Size && py >= 0 && py < LCD_Y_Size)
                        fb[py * LCD_X_Size + px] = 0xFF0000FF;
                }

            if (x == x2 && y == y2) break;
            int16_t e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
        }
    }
    uint16_t sx = xPos + (startX * scale + scale/2);
    uint16_t sy = yPos + (startY * scale + scale/2);
    for (int dy = -2; dy <= 2; dy++)
        for (int dx = -2; dx <= 2; dx++) {
            int32_t px = sx + dx, py = sy + dy;
            if (px >= 0 && px < LCD_X_Size && py >= 0 && py < LCD_Y_Size)
                fb[py * LCD_X_Size + px] = 0xFF00FF00;
        }

    uint16_t ex = xPos + (endX * scale + scale/2);
    uint16_t ey = yPos + (endY * scale + scale/2);
    for (int dy = -2; dy <= 2; dy++)
        for (int dx = -2; dx <= 2; dx++) {
            int32_t px = ex + dx, py = ey + dy;
            if (px >= 0 && px < LCD_X_Size && py >= 0 && py < LCD_Y_Size)
                fb[py * LCD_X_Size + px] = 0xFFFF0000;
        }

    SCB_CleanDCache();
}

static void solveMaze(uint32_t xPos, uint32_t yPos, JPEG_ConfTypeDef *jpegInfo)
{
    uint16_t startX = 0, startY = 0;
    uint16_t endX = 0, endY = 0;
    uint8_t found = 0;

    if (HAL_DMA2D_GetState(&DMA2D_Handle) == HAL_DMA2D_STATE_BUSY) {
        HAL_DMA2D_Abort(&DMA2D_Handle);
        HAL_Delay(10);
    }

    uint16_t topCandidates[MAZE_WIDTH];
    uint16_t bottomCandidates[MAZE_WIDTH];
    uint16_t leftCandidates[MAZE_HEIGHT];
    uint16_t rightCandidates[MAZE_HEIGHT];


    uint16_t topCount = 0, bottomCount = 0, leftCount = 0, rightCount = 0;

    for (uint16_t x = 0; x < MAZE_WIDTH; x++)
        if (mazeArray[0][x] == 1)
            topCandidates[topCount++] = x;

    for (uint16_t x = 0; x < MAZE_WIDTH; x++)
        if (mazeArray[MAZE_HEIGHT - 1][x] == 1)
            bottomCandidates[bottomCount++] = x;

    for (uint16_t y = 0; y < MAZE_HEIGHT; y++)
        if (mazeArray[y][0] == 1)
            leftCandidates[leftCount++] = y;

    for (uint16_t y = 0; y < MAZE_HEIGHT; y++)
        if (mazeArray[y][MAZE_WIDTH - 1] == 1)
            rightCandidates[rightCount++] = y;
    for (uint16_t i = 0; i < topCount && !found; i++)
    {
        for (uint16_t j = 0; j < bottomCount && !found; j++)
        {
            startX = topCandidates[i];
            startY = 0;
            endX = bottomCandidates[j];
            endY = MAZE_HEIGHT - 1;

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
            {
                found = 1;
            }
        }
    }

    for (uint16_t i = 0; i < topCount && !found; i++)
    {
        for (uint16_t j = 0; j < rightCount && !found; j++)
        {
            startX = topCandidates[i];
            startY = 0;
            endX = MAZE_WIDTH - 1;
            endY = rightCandidates[j];

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < topCount && !found; i++)
    {
        for (uint16_t j = 0; j < leftCount && !found; j++)
        {
            startX = topCandidates[i];
            startY = 0;
            endX = 0;
            endY = leftCandidates[j];

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < leftCount && !found; i++)
    {
        for (uint16_t j = 0; j < rightCount && !found; j++)
        {
            startX = 0;
            startY = leftCandidates[i];
            endX = MAZE_WIDTH - 1;
            endY = rightCandidates[j];

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < rightCount && !found; i++)
    {
        for (uint16_t j = 0; j < bottomCount && !found; j++)
        {
            startX = MAZE_WIDTH - 1;
            startY = rightCandidates[i];
            endX = bottomCandidates[j];
            endY = MAZE_HEIGHT - 1;

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < leftCount && !found; i++)
    {
        for (uint16_t j = 0; j < bottomCount && !found; j++)
        {
            startX = 0;
            startY = leftCandidates[i];
            endX = bottomCandidates[j];
            endY = MAZE_HEIGHT - 1;

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }
    for (uint16_t i = 0; i < rightCount && !found; i++)
    {
        for (uint16_t j = 0; j < leftCount && !found; j++)
        {
            startX = MAZE_WIDTH - 1;
            startY = rightCandidates[i];
            endX = 0;
            endY = leftCandidates[j];

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < rightCount && !found; i++)
    {
        for (uint16_t j = 0; j < topCount && !found; j++)
        {
            startX = MAZE_WIDTH - 1;
            startY = rightCandidates[i];
            endX = topCandidates[j];
            endY = 0;

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < bottomCount && !found; i++)
    {
        for (uint16_t j = 0; j < topCount && !found; j++)
        {
            startX = bottomCandidates[i];
            startY = MAZE_HEIGHT - 1;
            endX = topCandidates[j];
            endY = 0;

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < bottomCount && !found; i++)
    {
        for (uint16_t j = 0; j < leftCount && !found; j++)
        {
            startX = bottomCandidates[i];
            startY = MAZE_HEIGHT - 1;
            endX = 0;
            endY = leftCandidates[j];

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    for (uint16_t i = 0; i < bottomCount && !found; i++)
    {
        for (uint16_t j = 0; j < rightCount && !found; j++)
        {
            startX = bottomCandidates[i];
            startY = MAZE_HEIGHT - 1;
            endX = MAZE_WIDTH - 1;
            endY = rightCandidates[j];

            findPathBFS(startX, startY, endX, endY);
            if (prev[endY][endX] != 0xFFFF)
                found = 1;
        }
    }

    if (found)
    {
        SCB_InvalidateDCache();
        drawPathOnOriginal(startX, startY, endX, endY, xPos, yPos, jpegInfo);
        SCB_CleanInvalidateDCache();
    }
}
static uint8_t ReadJoystick(void)
{
    static uint32_t lastRead = 0;
    uint32_t currentTime = HAL_GetTick();
    uint8_t result = 0;

    if (currentTime - lastRead < 200) {
        return 0;
    }

    if(HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_6) == GPIO_PIN_RESET) {      // UP (PK6)
        result = 1;
        lastRead = currentTime;
    }
    else if(HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_3) == GPIO_PIN_RESET) { // DOWN (PK3)
        result = 2;
        lastRead = currentTime;
    }
    else if(HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_4) == GPIO_PIN_RESET) { // LEFT (PK4)
        result = 3;
        lastRead = currentTime;
    }
    else if(HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_5) == GPIO_PIN_RESET) { // RIGHT (PK5)
        result = 4;
        lastRead = currentTime;
    }

    return result;
}

static void MX_JOYSTICK_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOK_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);
}
int main(void)
{
  uint32_t xPos = 0, yPos = 0;
  uint8_t  lcd_status = BSP_ERROR_NONE;
  uint32_t file_error = 0, sd_detection_error = 0;

  /* System Init, System clock, voltage scaling and L1-Cache configuration are done by CPU1 (Cortex-M7)
     in the meantime Domain D2 is put in STOP mode(Cortex-M4 in deep-sleep)
  */

  /* Configure the MPU attributes as Write Through for SDRAM*/
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32H7xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  SystemClock_Config();

  /* When system initialization is finished, Cortex-M7 could wakeup (when needed) the Cortex-M4  by means of
     HSEM notification or by any D2 wakeup source (SEV,EXTI..)   */

  /* Initialize the LED3 (Red LED , set to On when error) */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  MX_JOYSTICK_GPIO_Init();
  /*##-1- JPEG Initialization ################################################*/
   /* Init the HAL JPEG driver */
  JPEG_Handle.Instance = JPEG;
  HAL_JPEG_Init(&JPEG_Handle);

  /*##-2- LCD Configuration ##################################################*/
  /* Initialize the LCD   */

  lcd_status = BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
  if(lcd_status != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  UTIL_LCD_SetFuncDriver(&LCD_Driver);
  UTIL_LCD_SetLayer(0);

/* Get the LCD Width and Height */
  BSP_LCD_GetXSize(0, &LCD_X_Size);
  BSP_LCD_GetYSize(0, &LCD_Y_Size);

  /* Cear LCD */
  UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);

  /*##-3- Link the micro SD disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /*##-4- Init the SD Card #################################################*/
    SD_Initialize();

    if(BSP_SD_IsDetected(0))
    {
      /*##-5- Register the file system object to the FatFs module ##############*/
      if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
      {
        /* Scan for JPEG files */
        ScanJpegFiles();

        if (fileCount > 0) {
          /* Display file menu */
          DisplayFileMenu();
          uint32_t lastJoyTime = 0;
          /* Main loop with joystick control */
          while (1)
          {
            uint8_t joy = ReadJoystick();
            uint32_t currentTime = HAL_GetTick();
            if (menuActive) {
                if (joy == 1 && currentTime - lastJoyTime > 250) { // UP
                    if (selectedFile > 0) {
                        selectedFile--;
                        DisplayFileMenu();
                        lastJoyTime = currentTime;
                    }
                }
                else if (joy == 2 && currentTime - lastJoyTime > 250) { // DOWN
                    if (selectedFile < fileCount - 1) {
                        selectedFile++;
                        DisplayFileMenu();
                        lastJoyTime = currentTime;
                    }
                }
                else if (joy == 4 && currentTime - lastJoyTime > 250) { // RIGHT
                    /* Open selected file */
                    lastJoyTime = currentTime;
                    OpenAndDisplayJpeg(selectedFile);
                }
            } else {
            	if (joy == 3 && currentTime - lastJoyTime > 250) { // LEFT
            	    menuActive = 1;
            	    DisplayFileMenu();
            	}
            }
            HAL_Delay(20);
          }
        } else {
          /* No JPEG files found */
          UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
          UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_RED);
          UTIL_LCD_SetFont(&Font16);
          UTIL_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)"No JPEG files found!", CENTER_MODE);
          UTIL_LCD_DisplayStringAt(0, LINE(12), (uint8_t *)"Please copy .jpg files to SD card", CENTER_MODE);
        }
      }
      else
      {
        sd_detection_error = 1;
      }
    }
    else
    {
      sd_detection_error = 1;
    }

    if((file_error != 0) || (sd_detection_error != 0))
    {
      /* Display example brief   */
      LCD_BriefDisplay();
      /* Display error brief   */
      LCD_FileErrorDisplay();
      Error_Handler();
    }
  }

  /* Infinite loop */
  while (1)
  {
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 400000000 (CM7 CPU Clock)
  *            HCLK(Hz)                       = 200000000 (CM4 CPU, AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 160
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

 /*
  Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
          (GPIO, SPI, FMC, QSPI ...)  when  operating at  high frequencies(please refer to product datasheet)
          The I/O Compensation Cell activation  procedure requires :
        - The activation of the CSI clock
        - The activation of the SYSCFG clock
        - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR
 */

  /*activate CSI clock mondatory for I/O Compensation Cell*/
  __HAL_RCC_CSI_ENABLE() ;

  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();
}

/**
  * @brief  Configure the MPU attributes as Write Through for External SDRAM.
  * @note   The Base Address is SDRAM_DEVICE_ADDR .
  *         The Configured Region Size is 32MB because same as SDRAM size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as WT for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = SDRAM_DEVICE_ADDR;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


/**
* @brief  CPU L1-Cache enable.
* @param  None
* @retval None
*/
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @brief  Display Example description.
  * @param  None
  * @retval None
  */
static void LCD_BriefDisplay(void)
{
  UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLUE);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_BLUE);
  UTIL_LCD_FillRect(0, 0, LCD_X_Size, 112, UTIL_LCD_COLOR_BLUE);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
  UTIL_LCD_DisplayStringAt(0, LINE(2), (uint8_t *)"JPEG Decoding from uSD Fatfs with DMA", CENTER_MODE);
  UTIL_LCD_SetFont(&Font16);
  UTIL_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"This example shows how to Decode (with DMA)", CENTER_MODE);
  UTIL_LCD_DisplayStringAt(0, LINE(6), (uint8_t *)"and  display a JPEG file", CENTER_MODE);
}

/**
  * @brief  Display File access error message.
  * @param  None
  * @retval None
  */
static void LCD_FileErrorDisplay(void)
{
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_WHITE);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_RED);
  UTIL_LCD_SetFont(&Font16);
  UTIL_LCD_DisplayStringAtLine(26, (uint8_t *)"     Unable to open JPEG file image.jpg");
  UTIL_LCD_DisplayStringAtLine(27, (uint8_t *)"     Please check that a jpeg file named image.jpg");
  UTIL_LCD_DisplayStringAtLine(28, (uint8_t *)"     is stored on the uSD");
}

/**
  * @brief  Copy the Decoded image to the display Frame buffer.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Pointer to destination buffer
  * @param  x: destination Horizontal offset.
  * @param  y: destination Vertical offset.
  * @param  xsize: image width
  * @param  ysize: image Height
  * @param  ChromaSampling : YCbCr Chroma sampling : 4:2:0, 4:2:2 or 4:4:4
  * @retval None
  */
static void DMA2D_CopyBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize, uint32_t ChromaSampling)
{

  uint32_t cssMode = DMA2D_CSS_420, inputLineOffset = 0;
  uint32_t destination = 0;

  if(ChromaSampling == JPEG_420_SUBSAMPLING)
  {
    cssMode = DMA2D_CSS_420;

    inputLineOffset = xsize % 16;
    if(inputLineOffset != 0)
    {
      inputLineOffset = 16 - inputLineOffset;
    }
  }
  else if(ChromaSampling == JPEG_444_SUBSAMPLING)
  {
    cssMode = DMA2D_NO_CSS;

    inputLineOffset = xsize % 8;
    if(inputLineOffset != 0)
    {
      inputLineOffset = 8 - inputLineOffset;
    }
  }
  else if(ChromaSampling == JPEG_422_SUBSAMPLING)
  {
    cssMode = DMA2D_CSS_422;

    inputLineOffset = xsize % 16;
    if(inputLineOffset != 0)
    {
      inputLineOffset = 16 - inputLineOffset;
    }
  }

  /*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
  DMA2D_Handle.Init.Mode         = DMA2D_M2M_PFC;
  DMA2D_Handle.Init.ColorMode    = DMA2D_OUTPUT_ARGB8888;
  DMA2D_Handle.Init.OutputOffset = LCD_X_Size - xsize;
  DMA2D_Handle.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
  DMA2D_Handle.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

  /*##-2- DMA2D Callbacks Configuration ######################################*/
  DMA2D_Handle.XferCpltCallback  = NULL;

  /*##-3- Foreground Configuration ###########################################*/
  DMA2D_Handle.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
  DMA2D_Handle.LayerCfg[1].InputAlpha = 0xFF;
  DMA2D_Handle.LayerCfg[1].InputColorMode = DMA2D_INPUT_YCBCR;
  DMA2D_Handle.LayerCfg[1].ChromaSubSampling = cssMode;
  DMA2D_Handle.LayerCfg[1].InputOffset = inputLineOffset;
  DMA2D_Handle.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
  DMA2D_Handle.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

  DMA2D_Handle.Instance          = DMA2D;

  /*##-4- DMA2D Initialization     ###########################################*/
  HAL_DMA2D_Init(&DMA2D_Handle);
  HAL_DMA2D_ConfigLayer(&DMA2D_Handle, 1);

  /*##-5-  copy the new decoded frame to the LCD Frame buffer ################*/
  destination = (uint32_t)pDst + ((y * LCD_X_Size) + x) * 4;

  HAL_DMA2D_Start(&DMA2D_Handle, (uint32_t)pSrc, destination, xsize, ysize);
  HAL_DMA2D_PollForTransfer(&DMA2D_Handle, 25);  /* wait for the previous DMA2D transfer to ends */
}

static void SD_Initialize(void)
{
  BSP_SD_Init(0);
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/**
  * @brief  On Error Handler.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  BSP_LED_On(LED3);
  while(1) { ; } /* Blocking on error */
}

/**
  * @}
  */

/**
  * @}
  */
