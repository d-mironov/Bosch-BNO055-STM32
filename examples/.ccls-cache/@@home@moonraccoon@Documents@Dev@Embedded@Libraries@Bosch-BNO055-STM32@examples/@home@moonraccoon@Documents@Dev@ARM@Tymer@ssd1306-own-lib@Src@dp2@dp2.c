#include "dp2.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>  // For memcpy

#if defined(DP2_USE_I2C)

void dp2_Reset(void) {
    /* for I2C - do nothing */
}

// Send a byte to the command register
void dp2_WriteCommand(uint8_t byte) {
    HAL_I2C_Mem_Write(&DP2_I2C_PORT, DP2_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

// Send data
void dp2_WriteData(uint8_t* buffer, size_t buff_size) {
    HAL_I2C_Mem_Write(&DP2_I2C_PORT, DP2_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}

#elif defined(DP2_USE_SPI)

void dp2_Reset(void) {
    // CS = High (not selected)
    HAL_GPIO_WritePin(DP2_CS_Port, DP2_CS_Pin, GPIO_PIN_SET);

    // Reset the OLED
    HAL_GPIO_WritePin(DP2_Reset_Port, DP2_Reset_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DP2_Reset_Port, DP2_Reset_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

// Send a byte to the command register
void dp2_WriteCommand(uint8_t byte) {
    HAL_GPIO_WritePin(DP2_CS_Port, DP2_CS_Pin, GPIO_PIN_RESET); // select OLED
    HAL_GPIO_WritePin(DP2_DC_Port, DP2_DC_Pin, GPIO_PIN_RESET); // command
    HAL_SPI_Transmit(&DP2_SPI_PORT, (uint8_t *) &byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(DP2_CS_Port, DP2_CS_Pin, GPIO_PIN_SET); // un-select OLED
}

// Send data
void dp2_WriteData(uint8_t* buffer, size_t buff_size) {
    HAL_GPIO_WritePin(DP2_CS_Port, DP2_CS_Pin, GPIO_PIN_RESET); // select OLED
    HAL_GPIO_WritePin(DP2_DC_Port, DP2_DC_Pin, GPIO_PIN_SET); // data
    HAL_SPI_Transmit(&DP2_SPI_PORT, buffer, buff_size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(DP2_CS_Port, DP2_CS_Pin, GPIO_PIN_SET); // un-select OLED
}

#else
#error "You should define DP2_USE_SPI or DP2_USE_I2C macro"
#endif


// Screenbuffer
static uint8_t DP2_Buffer[DP2_BUFFER_SIZE];

// Screen object
static DP2_t SSD1306;

/* Fills the Screenbuffer with values from a given buffer of a fixed length */
DP2_Error_t dp2_FillBuffer(uint8_t* buf, uint32_t len) {
    DP2_Error_t ret = DP2_ERR;
    if (len <= DP2_BUFFER_SIZE) {
        memcpy(DP2_Buffer,buf,len);
        ret = DP2_OK;
    }
    return ret;
}

// Initialize the oled screen
void dp2_Init(void) {
    // Reset OLED
    dp2_Reset();

    // Wait for the screen to boot
    HAL_Delay(100);

    // Init OLED
    dp2_SetDisplayOn(0); //display off

    dp2_WriteCommand(0x20); //Set Memory Addressing Mode
    dp2_WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                // 10b,Page Addressing Mode (RESET); 11b,Invalid

    dp2_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

#ifdef DP2_MIRROR_VERT
    dp2_WriteCommand(0xC0); // Mirror vertically
#else
    dp2_WriteCommand(0xC8); //Set COM Output Scan Direction
#endif

    dp2_WriteCommand(0x00); //---set low column address
    dp2_WriteCommand(0x10); //---set high column address

    dp2_WriteCommand(0x40); //--set start line address - CHECK

    dp2_SetContrast(0xFF);

#ifdef DP2_MIRROR_HORIZ
    dp2_WriteCommand(0xA0); // Mirror horizontally
#else
    dp2_WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef DP2_INVERSE_COLOR
    dp2_WriteCommand(0xA7); //--set inverse color
#else
    dp2_WriteCommand(0xA6); //--set normal color
#endif

// Set multiplex ratio.
#if (DP2_HEIGHT == 128)
    // Found in the Luma Python lib for SH1106.
    dp2_WriteCommand(0xFF);
#else
    dp2_WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
#endif

#if (DP2_HEIGHT == 32)
    dp2_WriteCommand(0x1F); //
#elif (DP2_HEIGHT == 64)
    dp2_WriteCommand(0x3F); //
#elif (DP2_HEIGHT == 128)
    dp2_WriteCommand(0x3F); // Seems to work for 128px high displays too.
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    dp2_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    dp2_WriteCommand(0xD3); //-set display offset - CHECK
    dp2_WriteCommand(0x00); //-not offset

    dp2_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
    dp2_WriteCommand(0xF0); //--set divide ratio

    dp2_WriteCommand(0xD9); //--set pre-charge period
    dp2_WriteCommand(0x22); //

    dp2_WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
#if (DP2_HEIGHT == 32)
    dp2_WriteCommand(0x02);
#elif (DP2_HEIGHT == 64)
    dp2_WriteCommand(0x12);
#elif (DP2_HEIGHT == 128)
    dp2_WriteCommand(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    dp2_WriteCommand(0xDB); //--set vcomh
    dp2_WriteCommand(0x20); //0x20,0.77xVcc

    dp2_WriteCommand(0x8D); //--set DC-DC enable
    dp2_WriteCommand(0x14); //
    dp2_SetDisplayOn(1); //--turn on SSD1306 panel

    // Clear screen
    dp2_Fill(DP2_Black);
    
    // Flush buffer to screen
    dp2_UpdateScreen();
    
    // Set default values for screen object
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;
    
    SSD1306.Initialized = 1;
}

// Fill the whole screen with the given color
void dp2_Fill(DP2_COLOR color) {
    /* Set memory */
    uint32_t i;

    for(i = 0; i < sizeof(DP2_Buffer); i++) {
        DP2_Buffer[i] = (color == DP2_Black) ? 0x00 : 0xFF;
    }
}

// Write the screenbuffer with changed to the screen
void dp2_UpdateScreen(void) {
    // Write data to each page of RAM. Number of pages
    // depends on the screen height:
    //
    //  * 32px   ==  4 pages
    //  * 64px   ==  8 pages
    //  * 128px  ==  16 pages
    for(uint8_t i = 0; i < DP2_HEIGHT/8; i++) {
        dp2_WriteCommand(0xB0 + i); // Set the current RAM page address.
        dp2_WriteCommand(0x00 + DP2_X_OFFSET_LOWER);
        dp2_WriteCommand(0x10 + DP2_X_OFFSET_UPPER);
        dp2_WriteData(&DP2_Buffer[DP2_WIDTH*i],DP2_WIDTH);
    }
}

//    Draw one pixel in the screenbuffer
//    X => X Coordinate
//    Y => Y Coordinate
//    color => Pixel color
void dp2_DrawPixel(uint8_t x, uint8_t y, DP2_COLOR color) {
    if(x >= DP2_WIDTH || y >= DP2_HEIGHT) {
        // Don't write outside the buffer
        return;
    }
   
    // Draw in the right color
    if(color == DP2_White) {
        DP2_Buffer[x + (y / 8) * DP2_WIDTH] |= 1 << (y % 8);
    } else { 
        DP2_Buffer[x + (y / 8) * DP2_WIDTH] &= ~(1 << (y % 8));
    }
}

// Draw 1 char to the screen buffer
// ch       => char om weg te schrijven
// Font     => Font waarmee we gaan schrijven
// color    => DP2_Black or DP2_White
char dp2_WriteChar(char ch, FontDef Font, DP2_COLOR color) {
    uint32_t i, b, j;
    
    // Check if character is valid
    if (ch < 32 || ch > 126)
        return 0;
    
    // Check remaining space on current line
    if (DP2_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
        DP2_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
    {
        // Not enough space on current line
        return 0;
    }
    
    // Use the font to write
    for(i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for(j = 0; j < Font.FontWidth; j++) {
            if((b << j) & 0x8000)  {
                dp2_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (DP2_COLOR) color);
            } else {
                dp2_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (DP2_COLOR)!color);
            }
        }
    }
    
    // The current space is now taken
    SSD1306.CurrentX += Font.FontWidth;
    
    // Return written char for validation
    return ch;
}

// Write full string to screenbuffer
char dp2_WriteString(char* str, FontDef Font, DP2_COLOR color) {
    // Write until null-byte
    while (*str) {
        if (dp2_WriteChar(*str, Font, color) != *str) {
            // Char could not be written
            return *str;
        }
        
        // Next char
        str++;
    }
    
    // Everything ok
    return *str;
}

// Position the cursor
void dp2_SetCursor(uint8_t x, uint8_t y) {
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

// Draw line by Bresenhem's algorithm
void dp2_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, DP2_COLOR color) {
  int32_t deltaX = abs(x2 - x1);
  int32_t deltaY = abs(y2 - y1);
  int32_t signX = ((x1 < x2) ? 1 : -1);
  int32_t signY = ((y1 < y2) ? 1 : -1);
  int32_t error = deltaX - deltaY;
  int32_t error2;
    
  dp2_DrawPixel(x2, y2, color);
    while((x1 != x2) || (y1 != y2))
    {
    dp2_DrawPixel(x1, y1, color);
    error2 = error * 2;
    if(error2 > -deltaY)
    {
      error -= deltaY;
      x1 += signX;
    }
    else
    {
    /*nothing to do*/
    }
        
    if(error2 < deltaX)
    {
      error += deltaX;
      y1 += signY;
    }
    else
    {
    /*nothing to do*/
    }
  }
  return;
}
//Draw polyline
void dp2_Polyline(const DP2_VERTEX *par_vertex, uint16_t par_size, DP2_COLOR color) {
  uint16_t i;
  if(par_vertex != 0){
    for(i = 1; i < par_size; i++){
      dp2_Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
    }
  }
  else
  {
    /*nothing to do*/
  }
  return;
}
/*Convert Degrees to Radians*/
static float dp2_DegToRad(float par_deg) {
    return par_deg * 3.14 / 180.0;
}
/*Normalize degree to [0;360]*/
static uint16_t dp2_NormalizeTo0_360(uint16_t par_deg) {
  uint16_t loc_angle;
  if(par_deg <= 360)
  {
    loc_angle = par_deg;
  }
  else
  {
    loc_angle = par_deg % 360;
    loc_angle = ((par_deg != 0)?par_deg:360);
  }
  return loc_angle;
}
/*DrawArc. Draw angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle in degree
 * sweep in degree
 */
void dp2_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, DP2_COLOR color) {
    #define CIRCLE_APPROXIMATION_SEGMENTS 36
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1,xp2;
    uint8_t yp1,yp2;
    uint32_t count = 0;
    uint32_t loc_sweep = 0;
    float rad;
    
    loc_sweep = dp2_NormalizeTo0_360(sweep);
    
    count = (dp2_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;
    while(count < approx_segments)
    {
        rad = dp2_DegToRad(count*approx_degree);
        xp1 = x + (int8_t)(sin(rad)*radius);
        yp1 = y + (int8_t)(cos(rad)*radius);    
        count++;
        if(count != approx_segments)
        {
            rad = dp2_DegToRad(count*approx_degree);
        }
        else
        {            
            rad = dp2_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t)(sin(rad)*radius);
        yp2 = y + (int8_t)(cos(rad)*radius);    
        dp2_Line(xp1,yp1,xp2,yp2,color);
    }
    
    return;
}
//Draw circle by Bresenhem's algorithm
void dp2_DrawCircle(uint8_t par_x,uint8_t par_y,uint8_t par_r,DP2_COLOR par_color) {
  int32_t x = -par_r;
  int32_t y = 0;
  int32_t err = 2 - 2 * par_r;
  int32_t e2;

  if (par_x >= DP2_WIDTH || par_y >= DP2_HEIGHT) {
    return;
  }

    do {
      dp2_DrawPixel(par_x - x, par_y + y, par_color);
      dp2_DrawPixel(par_x + x, par_y + y, par_color);
      dp2_DrawPixel(par_x + x, par_y - y, par_color);
      dp2_DrawPixel(par_x - x, par_y - y, par_color);
        e2 = err;
        if (e2 <= y) {
            y++;
            err = err + (y * 2 + 1);
            if(-x == y && e2 <= x) {
              e2 = 0;
            }
            else
            {
              /*nothing to do*/
            }
        }
        else
        {
          /*nothing to do*/
        }
        if(e2 > x) {
          x++;
          err = err + (x * 2 + 1);
        }
        else
        {
          /*nothing to do*/
        }
    } while(x <= 0);

    return;
}

//Draw rectangle
void dp2_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, DP2_COLOR color) {
  dp2_Line(x1,y1,x2,y1,color);
  dp2_Line(x2,y1,x2,y2,color);
  dp2_Line(x2,y2,x1,y2,color);
  dp2_Line(x1,y2,x1,y1,color);

  return;
}

//Draw bitmap - ported from the ADAFruit GFX library

void dp2_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, DP2_COLOR color)
{
    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    if (x >= DP2_WIDTH || y >= DP2_HEIGHT) {
        return;
    }

    for (uint8_t j = 0; j < h; j++, y++) {
        for (uint8_t i = 0; i < w; i++) {
            if (i & 7)
                byte <<= 1;
            else
                byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            if (byte & 0x80)
                dp2_DrawPixel(x + i, y, color);
        }
    }
    return;
}

void dp2_SetContrast(const uint8_t value) {
    const uint8_t kSetContrastControlRegister = 0x81;
    dp2_WriteCommand(kSetContrastControlRegister);
    dp2_WriteCommand(value);
}

void dp2_SetDisplayOn(const uint8_t on) {
    uint8_t value;
    if (on) {
        value = 0xAF;   // Display on
        SSD1306.DisplayOn = 1;
    } else {
        value = 0xAE;   // Display off
        SSD1306.DisplayOn = 0;
    }
    dp2_WriteCommand(value);
}

uint8_t dp2_GetDisplayOn() {
    return SSD1306.DisplayOn;
}
