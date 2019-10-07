#include "mcp23x17.h"
#include "panel.h"
#include <string.h>
#include "stm32f1xx_hal.h"

const uint8_t fonts[] = {
  0b00111111,//0 
  0b00000110,//1
  0b01011011,//2
  0b01001111,//3
  0b01100110,//4
  0b01101101,//5
  0b01111101,//6
  0b00100111,//7
  0b01111111,//8
  0b01100111,//9
  0b01110111,//A
  0b01111100,//b
  0b01011000,//c
  0b01011110,//d
  0b01111001,//E
  0b01110001,//F
  //////////////
  0b01101111,//g      //0x10
  0b01110100,//h      //0x11
  0b00010000,//i      //0x12
  0b00011110,//J      //0x13
  0b01110101,//K      //0x14
  0b00111000,//L      //0x15
  0b00110111,//M      //0x16
  0b01010100,//n      //0x17
  0b01011100,//o      //0x18
  0b01110011,//P      //0x19
  0b01100111,//q      //0x1A
  0b01010000,//r      //0x1B
  0b01101100,//S      //0x1C
  0b01111000,//t      //0x1D
  0b00011100,//u      //0x1E
  0b00111110,//V      //0x1F
  0b01111110,//w      //0x20
  0b01110110,//x      //0x21
  0b01101110,//y      //0x22
  0b00011011,//z      //0x23
  0b00001000,//_      //0x24
  0b10000000,//.      //0x25
  0b01000000,//-      //0x26
  0b00100000,//~      //0x27
  //0x24-0x3F
               0,0,0,0,0,0,0,0,0,0
  ,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

//blank期間が5%は無いとちらつく
const uint16_t illiuminate_duty[4] = {950,800,500,50};

typedef struct {
  uint8_t  last;
  uint16_t time;
} buttonstate;

enum MODE {
  TACHOMETER,
  ILLUMINATION,
};

const uint8_t modes[2][4] = {
   {0x19,0x1B,0x30,0x30,} //__rP
  ,{0x15,0x12,0x30,0x30,} //__iL
};

enum VIEW {
  NORMAL,
  CONFIG,
  SHOWMODE,
  POWEROFF,
};

struct  {
  I2C_HandleTypeDef *hI2C;
  TIM_HandleTypeDef *hBTN;
  TIM_HandleTypeDef *hPWM;
  TIM_HandleTypeDef *hBUZ;
  uint16_t mcp23017addr;
  uint8_t segment[5];
  uint8_t digit;
  
  buttonstate button;
  
  enum MODE mode;
  uint16_t modeTime;
  enum VIEW view;
  uint16_t viewTime;

  uint16_t limit;
  uint16_t illum;
  uint16_t rpm;
} ctx;

int panel_initialize(
  TIM_HandleTypeDef *b, TIM_HandleTypeDef *p, TIM_HandleTypeDef *z
, I2C_HandleTypeDef *h, uint16_t a) {
 
  ctx.mode = TACHOMETER;
  ctx.view = SHOWMODE;
  ctx.limit = 6800;
  ctx.illum = 3;
  ctx.rpm = 0;
  ctx.hBTN = b;
  ctx.hPWM = p;
  ctx.hBUZ = z;
  ctx.hI2C = h;
  ctx.mcp23017addr = a;
  ctx.digit = 0;

  memset(ctx.segment,0x00,5);

  ctx.button.last = 0;
  ctx.button.time = 0;

  //Initialize 23017
  uint8_t IOCON_DEF = VAL23X17_IOCON_BNK1|VAL23X17_IOCON_MIRR_NO|VAL23X17_IOCON_INTACT_HIGH;
  uint8_t data[16];
  
  //Changed bank to 1, No Mirror interrupt, Transmit by Sequencial, thru control enabled
  data[0] = IOCON_DEF;
  if (HAL_I2C_Mem_Write(
     ctx.hI2C
    ,ctx.mcp23017addr
    ,REG23x17_IOCONA_BNK0,1,data,1
    ,1000
  ) != HAL_OK) 
  {
    return ERROR;
  }
  //access register sequencial for PORT A
  data[ 0] = 0x00;        //IODIRA   | PORT A ARE SET OUTPUT
  data[ 1] = 0x00;        //IPOLA    | INPUT POLAR ARE NOT REVERSED
  data[ 2] = 0x00;        //GPINTENA | DISABLED input value changed event
  data[ 3] = 0x00;        //DEFVALA  | ALL CLEARED (not used,GPINTEN are disabled)
  data[ 4] = 0x00;        //INTCONA  | ALL CLEARED (not used,GPINTEN are disabled)
  data[ 5] = IOCON_DEF;   //IOCON    | default
  data[ 6] = 0x00;        //GPPUA    | NO PULL UP
  data[ 7] = 0x00;        //INTFA    | READONLY ignored
  data[ 8] = 0x00;        //INTCAPA  | READONLY ignored
  data[ 9] = 0xFF;        //GPIOA    | PORT A ALL 1
  data[10] = 0xFF;        //OLATA    | LATCH A ALL 1
  //write PORT A setting register
  if (HAL_I2C_Mem_Write(
     ctx.hI2C
    ,ctx.mcp23017addr
    ,REG23x17_IODIRA_BNK1,1,data,11
    ,1000) != HAL_OK) 
  {
    return ERROR;
  }

  //access register sequencial for PORT B
  data[ 0] = 0xC0;        //IODIRB   | PORT B 76 ARE INPUT, (5)43210 ARE OUTPUT
  data[ 1] = 0xC0;        //IPOLB    | INPUT POLAR MSB 2bit are REVERSED other not reversed
  data[ 2] = 0xC0;        //GPINTENB | MSB 2bit are enabled interruption by input
  data[ 3] = 0xC0;        //DEFVALB  | input value default 0 other not used
  data[ 4] = 0x00;        //INTCONB  | MSB 2bit are compared changes from PREVIOUS port val 
  data[ 5] = IOCON_DEF;   //IOCON    | default
  data[ 6] = 0xC0;        //GPPUB    | MSB 2bit pulled up 
  data[ 7] = 0x00;        //INTFB    | READONLY ignored
  data[ 8] = 0x00;        //INTCAPB  | READONLY ignored
  data[ 9] = 0x01;        //GPIOB    | PORT B SET 1
  data[10] = 0x01;        //OLATB    | LATCH B SET 1
  //write PORT A setting register
  if (HAL_I2C_Mem_Write(
     ctx.hI2C
    ,ctx.mcp23017addr
    ,REG23x17_IODIRB_BNK1,1,data,11
    ,1000) != HAL_OK) 
  {
    return ERROR;
  }
  
  panel_set_illum();

  return SUCCESS;
}


int panel_update_view() {
  
  //ブザーの制御
  switch (ctx.mode) {
  case TACHOMETER:
    if (ctx.limit < ctx.rpm) {
      panel_set_buzz(1);
    } else {
      panel_set_buzz(0);
    }
    break;
  default:
    panel_set_buzz(0);
    break;
  }

  if (ctx.digit > 0) return SUCCESS; //ここから↓は1/60secごとに呼ばれるように
  
  uint8_t bcd[4];
  switch (ctx.view) {
  case NORMAL:
    switch (ctx.mode) {
    case TACHOMETER:
      decimalToBCD(ctx.rpm,bcd);
      panel_setsegment(bcd,0b00000000);
      break;
    case ILLUMINATION:
      decimalToBCD(1000-illiuminate_duty[ctx.illum],bcd);
      panel_setsegment(bcd,0b00000000);
      break;
    default:
      panel_set_buzz(0);
      break;
    }
    break;
  case CONFIG:
    switch (ctx.mode) {
    case TACHOMETER:
      decimalToBCD(ctx.limit,bcd);
      panel_setsegment(bcd,0b01000000);
      break;
    case ILLUMINATION:
      decimalToBCD(ctx.illum,bcd);
      panel_setsegment(bcd,0b01000000);
      break;
    default:
      break;
    }
    break;
  case SHOWMODE:
    panel_setsegment(modes[ctx.mode],0b00110000);
    break;
  case POWEROFF:
    break;
  default:
    break;
  }

  return SUCCESS;
}


//value = fonts index
//option = 0b[-][dash][colonL][colonH][dot4][dot3][dot2][dot1]
int panel_setsegment(const uint8_t value[], const uint8_t option) {
  for (int i=0; i<4; i++) {
    uint8_t f = fonts[0x3f&value[i]];
    uint8_t o = (0x01&(option>>i))?0x80:0;
    ctx.segment[i]=f | o;
  }
  ctx.segment[4]=0x07&(option>>4);
  return SUCCESS;
}

int panel_refresh(void) {
  //カソード側書込み
  uint8_t pos = ctx.segment[ctx.digit];
  uint8_t seg = ~pos;
  //data[1] = 0x00;
  if (HAL_I2C_Mem_Write(
      ctx.hI2C
     ,ctx.mcp23017addr
     ,REG23x17_OLATA_BNK1,1,&seg,1
     ,1000) != HAL_OK) 
  {
    return ERROR;
  }
  //点灯
  uint8_t dig = 0x01 << ctx.digit;
  if (HAL_I2C_Mem_Write(
      ctx.hI2C
     ,ctx.mcp23017addr
     ,REG23x17_OLATB_BNK1,1,&dig,1
     ,1000) != HAL_OK) 
  {
    return ERROR;
  }
   ctx.digit = ++ctx.digit > 4 ? 0 : ctx.digit;

  return SUCCESS;
}

int panel_blank(void) {
  //消灯
  uint8_t dig  = 0x00;
  if (HAL_I2C_Mem_Write(
     ctx.hI2C
    ,ctx.mcp23017addr
    ,REG23x17_GPIOB_BNK1,1,&dig,1
    ,1000) != HAL_OK) 
  {
    return ERROR;
  }
  return SUCCESS;
}

int panel_update_buttons() {

  uint8_t data;

  if (HAL_I2C_Mem_Read(
   ctx.hI2C,ctx.mcp23017addr
    ,REG23x17_INTCAPB_BNK1,1
    ,&data,1,1000) != HAL_OK) {
    return ERROR;    
  }
  data = (0xC0&data) >> 6;
  uint16_t diff = __HAL_TIM_GET_COUNTER(ctx.hBTN) >> 1;
  switch(ctx.button.last) {
  case 0b00:
    switch(data) {
      case 0b01: //⬆️⬇️00->10 判定:⬆️   in
      case 0b10: //⬆️⬇️00->01 判定:⬇️   in
      case 0b11: //⬆️⬇️00->11 判定:⬆️⬇️ in
        ctx.button.time += diff;
        ctx.button.last = data;
        ctx.button.time = 0;
        __HAL_TIM_SET_COUNTER(ctx.hBTN,0);
        break;
      default: //変わってなかった(割り込みの取りこぼし対応用))
        ctx.button.time = ctx.button.time >= 5000 ? 0 : ctx.button.time+1000;
        break;
    }
    break;
  case 0b01:
    switch(data) {
      case 0b00: //⬆️⬇️10->00 判定:⬆️out
      case 0b10: //⬆️⬇️10->01 判定:⬆️out⬇️in
        if (ctx.button.time + diff > 3000) {
          panel_toggle_mode(1);
        } else {
           panel_set_config(1);
        }
      case 0b11: //⬆️⬇️10->11 判定:⬆️⬇️   in
        ctx.button.time += diff;
        ctx.button.last = data;
        ctx.button.time = 0;
        __HAL_TIM_SET_COUNTER(ctx.hBTN,0);
        break;
      default: //変わってなかった(割り込みの取りこぼし対応用))
        ctx.button.time = ctx.button.time >= 5000 ? 1000 : ctx.button.time+1000;
        break;
    }
    break;
  case 0b10:
    switch(data) {
      case 0b00: //⬆️⬇️01->00 判定:⬇️out
      case 0b01: //⬆️⬇️01->10 判定:⬇️out⬆️in
        if (ctx.button.time + diff > 3000) {
          panel_toggle_mode(-1);
        } else {
          panel_set_config(-1);
        }
      case 0b11: //⬆️⬇️01->11 判定:⬆️⬇️   in
        ctx.button.time += diff;
        ctx.button.last = data;
        ctx.button.time = 0;
        __HAL_TIM_SET_COUNTER(ctx.hBTN,0);
        break;
      default: //変わってなかった(割り込みの取りこぼし対応用))
        ctx.button.time = ctx.button.time >= 5000 ? 1000 : ctx.button.time+1000;
        break;
    }
    break;
  case 0b11:
    switch(data) {
      case 0b00: //⬆️⬇️11->00 判定:⬆️⬇️   out
      case 0b01: //⬆️⬇️11->10 判定:⬆️⬇️   out
      case 0b10: //⬆️⬇️11->01 判定:⬆️⬇️   out
        ctx.button.time += diff;
        ctx.button.last = data;
        ctx.button.time = 0;
        __HAL_TIM_SET_COUNTER(ctx.hBTN,0);
        break;
      default: //変わってなかった(割り込みの取りこぼし対応用))
        ctx.button.time = ctx.button.time >= 5000 ? 1000 : ctx.button.time+1000;
        break;
    }
    break;
  default:
    break;
  }
  return SUCCESS;
}

int panel_set_config(int dir) {
  panel_trigger_config();
  switch (ctx.mode) {
  case ILLUMINATION:
    if (dir > 0) { 
      ctx.illum= (ctx.illum+1 > 3) ? 3 : ctx.illum+1;
    }
    if (dir < 0) { 
      ctx.illum = (ctx.illum == 0) ? 0 : ctx.illum-1;
    }
    panel_set_illum();
    break;
  case TACHOMETER:
    if (dir > 0) { 
      ctx.limit = (ctx.limit+50 > 9950) ? 9950 : ctx.limit+50;
    }
    if (dir < 0) { 
      ctx.limit = (ctx.limit-50 <= 0  ) ? 0    : ctx.limit-50;
    }
    break;
  }
  return SUCCESS;
}

void panel_trigger_config() {
  ctx.view = CONFIG;
  ctx.viewTime = 0;
}

int panel_toggle_mode(int dir) {
  if (dir > 0) { 
    ctx.mode = (ctx.mode+1 >= ILLUMINATION) ? ILLUMINATION : ctx.mode+1;
    ctx.modeTime = 0;
    ctx.view = SHOWMODE;
    ctx.viewTime = 0;
  }
  if (dir < 0) { 
    ctx.mode = (ctx.mode-1 <= TACHOMETER) ? TACHOMETER : ctx.mode-1;
    ctx.modeTime = 0;
    ctx.view = SHOWMODE;
    ctx.viewTime = 0;
  }
  return SUCCESS;
}

void panel_measure_transient() {
  ctx.viewTime = ctx.viewTime >= 10 ? 0: ctx.viewTime+1;
  ctx.modeTime = ctx.modeTime >= 60 ? 0: ctx.modeTime+1;
  switch (ctx.view) {
  case NORMAL:
    break;
  case CONFIG:
    if (ctx.viewTime > 5) {
      ctx.viewTime = 0;
      ctx.view = NORMAL;
      //Todo: この時に設定値をセーブ
    }
  case SHOWMODE:
    if (ctx.viewTime > 1) {
      ctx.viewTime = 0;
      ctx.view = NORMAL;
    }
    break;
  default:
    break;
  }
  switch (ctx.mode) {
    case TACHOMETER:
      break;
    case ILLUMINATION:
      if (ctx.modeTime > 10) {
        ctx.modeTime = 0;
        ctx.mode = TACHOMETER;
      }
  }
}

void panel_set_illum() {
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = illiuminate_duty[ctx.illum];
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(ctx.hPWM,&sConfigOC,TIM_CHANNEL_1);
}

int panel_set_buzz(int on) {
  static int status = 0;
  
  if (status != on) {
    if ( on ) {
      if (HAL_TIM_PWM_Start(ctx.hBUZ,TIM_CHANNEL_4)!=HAL_OK) {
        return ERROR;
      }
    } else {
      if (HAL_TIM_PWM_Stop(ctx.hBUZ,TIM_CHANNEL_4)!=HAL_OK) {
        return ERROR;
      }
    }
    status = on;
  }  

  return SUCCESS;
}

void panel_updaterpm(uint16_t rpm) {
  ctx.rpm = rpm;
}

int decimalToBCD(uint16_t s, uint8_t d[]) {
  uint8_t d0 = s % 10;
  uint8_t d1 = s <= 10   ? 0x30 : (uint8_t)(s / 10 % 10);
  uint8_t d2 = s <= 100  ? 0x30 : (uint8_t)(s / 100 % 10);
  uint8_t d3 = s <= 1000 ? 0x30 : (uint8_t)(s / 1000 % 10);
  d[0] = d0;
  d[1] = d1;
  d[2] = d2;
  d[3] = d3;

  return SUCCESS;
}
