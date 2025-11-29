#include <unistd.h>
#include <stdio.h>

#ifndef __SYSTEM_INFO__
#define __SYSTEM_INFO__

#define DE10LITE 1 // change to 0 for CPUlator or DE1-SoC, 1 for DE10-Lite

#if DE10LITE
 #define BOARD				"DE10-Lite"
 #define MAX_X		160
 #define MAX_Y		120
 #define YSHIFT		  8
#else
 #define MAX_X		320
 #define MAX_Y		240
 #define YSHIFT		  9
#endif

#define SDRAM_BASE			0x00000000
#define SDRAM_END			0x03FFFFFF
#define FPGA_PIXEL_BUF_BASE		0x08000000
#define FPGA_PIXEL_BUF_END		0x0800FFFF
#define FPGA_CHAR_BASE			0x09000000
#define FPGA_CHAR_END			0x09001FFF

#define LED_BASE			0xFF200000
#define LEDR_BASE			0xFF200000
#define HEX3_HEX0_BASE			0xFF200020
#define HEX5_HEX4_BASE			0xFF200030
#define SW_BASE				0xFF200040
#define KEY_BASE			0xFF200050

#endif

typedef uint16_t pixel_t;
volatile pixel_t *pVGA = (pixel_t *)FPGA_PIXEL_BUF_BASE;

// fpga stuff 
volatile uint32_t *pSWITCH = (uint32_t *)SW_BASE;
volatile uint32_t *pKEY   = (uint32_t *)KEY_BASE;
volatile uint32_t *pHEXLO = (uint32_t *)HEX3_HEX0_BASE;
volatile uint32_t *pHEXHI = (uint32_t *)HEX5_HEX4_BASE;

const pixel_t blk = 0x0000; 
const pixel_t wht = 0xFFFF;
const pixel_t red = 0xF800;
const pixel_t grn = 0x07E0;
const pixel_t blu = 0x001f;


#define MAX_SCORE 9
#define SPEED 100000

// sevenseg display options
const uint8_t seg7[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};

// functions to draw on the screen
void drawPixel(int y, int x, pixel_t colour) {
    *(pVGA + (y<<YSHIFT) + x) = colour;
}

pixel_t readPixel(int y, int x) {
    return *(pVGA + (y<<YSHIFT) + x);
}

void rect(int y1, int y2, int x1, int x2, pixel_t c) {
    if (y1<0) 
        y1=0; 

    if (x1<0)
        x1=0;

    if (y2>MAX_Y)
        y2=MAX_Y; 

    if (x2>MAX_X)
        x2=MAX_X;

    for (int y=y1;y<y2;y++) 
        for (int x=x1;x<x2;x++) 
            drawPixel(y,x,c);
}

void clearScreen(pixel_t c) { 
	rect(0,MAX_Y,0,MAX_X,c); 
}

void delay(int N) { // read volatile memory location to waste time
	for (int i=0;i<N;i++) 
		*pVGA; 
	}

// displays score on 7 seg display
void displayScore(int p0, int p1) {
    uint32_t low = 0;
    low |= seg7[p0] << 0;  // writes 8 bits to HEX 0 
    low |= seg7[p1] << 8;  // concatenates 8 bits onto existing bits 
    *pHEXLO = low; // the top 16 bits are empty (HEX3/2 are fed 0s while 0/1 are fed player and robot vals)
    *pHEXHI = 0; // set hex 4/5 to 0
}

// movement functions 
enum Dir { UP=0, RIGHT=1, DOWN=2, LEFT=3 };
enum Dir turnLeft(enum Dir d)  {
     return (d+3)&3; 
    } // keeps it between 0 and 3
enum Dir turnRight(enum Dir d) { 
    return (d+1)&3; 
} // keeps it between 0 and 3


// indicates (x,y) position and increments with output of direction
void stepPos(int *y, int *x, enum Dir d) {
    if (d==UP) (*y)--; else if (d==DOWN) (*y)++;
    else if (d==LEFT) (*x)--; else (*x)++;
}


// check if we are inside the playable area
int insidePlayable(int y,int x,int b) {
    return (y>=b && y<MAX_Y-b && x>=b && x<MAX_X-b);
}


// create struct for player info
typedef struct {
    int y,x; // pos
    enum Dir dir; // current dir
    pixel_t colour; // colour
    int alive; // alive (1) dead (0)
} Player;

int detectObstacleAhead(int y,int x) {
    if (y<0||y>=MAX_Y||x<0||x>=MAX_X)       
	return 1; // there is an obstacle
    return (readPixel(y,x) != blk); // obstacle if not black
}

// button
void waitForKeyRelease(void) {
  while ( ((*pKEY) & 0x3) != 0x3 ) { // 0x3 reads only bottom 2 bits (key 0/1) --> makes sure that both keys aren't pressed (0x3)
        for (volatile int i=0;i<1000;i++) // delays the inputs so that there is no bounce and inputs can be properly processed (only one turn per press)
            *pVGA;
    }
}

void humanDecide(Player *pl) {
    static uint32_t prev = 0x3;  // assume released
    uint32_t now = *pKEY;

    int v0 = !(now & 0x1); // check if key 0 is pressed
    int v1 = !(now & 0x2); // check if key 1 is pressed
    int w0 = !(prev & 0x1); // check if key 0 was pressed
    int w1 = !(prev & 0x2); // check if key 1 was pressed

    if (v0 && !w0) pl->dir = turnLeft(pl->dir); // if key 0 is pressed and wasn't pressed before, turn left
    if (v1 && !w1) pl->dir = turnRight(pl->dir); // if key 1 is pressed and wasn't pressed before, turn right

    prev = now; // update previous state to current state 
}

// robot logic 
void robotDecide(Player *bot) {
    int y1=bot->y, x1=bot->x; 
    stepPos(&y1,&x1,bot->dir); // assign 1 step ahead of the robot's current location 
    int y2=y1, x2=x1; 
    stepPos(&y2,&x2,bot->dir); // assign 2 steps ahead of the robot's current location

    if (!(detectObstacleAhead(y1,x1)||detectObstacleAhead(y2,x2))) // if no obstacle ahead, dont turn 
        return;

    enum Dir L=turnLeft(bot->dir); // if straight not safe, check left 
    enum Dir R=turnRight(bot->dir); // if straight not safe, check right

    int yl=bot->y, xl=bot->x; // check left if left turn
    stepPos(&yl,&xl,L);
    int yr=bot->y, xr=bot->x; // check right if right turn
    stepPos(&yr,&xr,R);

    if (!detectObstacleAhead(yl,xl)) // if left is clear, turn left
        bot->dir=L;
    else if (!detectObstacleAhead(yr,xr)) // if right is clear, turn right
        bot->dir=R; 
}
 // if neither, keep going straight

int speedMod(void) {
    uint32_t sw = *pSWITCH; // read switch values
    int speedSetting = sw & 0x3FF; 
    int newSpeed;

    switch (speedSetting) {
        case 0:
            newSpeed = 1000000; 
            break;
        case 1: 
            newSpeed = 500000; 
            break; 
        case 2: 
            newSpeed = 250000; 
                break;
        case 4: 
            newSpeed = 150000; 
                break;
        case 8: 
            newSpeed = 75000;  
                break;
        case 16: 
            newSpeed = 50000;  
                break;
        case 32: 
            newSpeed = 25000;  
                break;
        case 64: 
            newSpeed = 10000;  
                break;
        case 128: 
            newSpeed = 5000;   
                break; 
        case 256: 
            newSpeed = 2500;   
                break;
        case 512: 
            newSpeed = 1250;   
                break;  
        default: 
            newSpeed = SPEED; 
                break;
    }

    return newSpeed;
}

/*void set_mtimer( volatile uint32_t *time_ptr, uint64_t new_time64 )
{
    *(time_ptr+0) = (uint32_t)0;
    *(time_ptr+1) = (uint32_t)(new_time64>>32);
    *(time_ptr+0) = (uint32_t)new_time64;
}

uint64_t get_mtimer( volatile uint32_t *time_ptr )
{
    uint32_t mtime_h, mtime_l;
    do {
        mtime_h = *(time_ptr+1);
        mtime_l = *(time_ptr+0);
    } while( mtime_h != *(time_ptr+1) );
    return ((uint64_t)mtime_h << 32) | mtime_l;
}

void setup_mtimecmp()
{
    uint64_t mtime64 = get_mtimer( mtime_ptr );
    mtime64 = (mtime64/PERIOD+1) * PERIOD;
    set_mtimer( mtime_ptr+2, mtime64 );
}

void mtimer_ISR(void)
{
    uint64_t mtimecmp64 = get_mtimer( mtime_ptr+2 );
    mtimecmp64 += PERIOD;
    set_mtimer( mtime_ptr+2, mtimecmp64 );

    tick = 1;
    counter = counter + 1; // debug
}

void key_ISR()
{
    volatile uint32_t *KEY_ptr = (uint32_t *)KEY_BASE;

    static uint32_t prev = 0x3;
    uint32_t now = *KEY_ptr;

    int edge = (~now) & prev; // detect new press

    edgePress = edge; // save globally

    if (edge & 0x1) human.dir = turnLeft(human.dir);
    if (edge & 0x2) human.dir = turnRight(human.dir);

    prev = now;
    *KEY_ptr = 0xF; // clear edgecapture
}

*/
 // main
int main() {

   /*setup_mtimecmp();       // start periodic interrupts
    setup_cpu_irqs(0x80 | 0x02);   // enable mtimer IRQ only */

    printf("RACERS, START YOUR ENGINES!!\n");
    clearScreen(blk); // set screen to blk

    // draw border
    const int border = 2;
    rect(0,border,0,MAX_X,wht);
    rect(MAX_Y-border,MAX_Y,0,MAX_X,wht);
    rect(0,MAX_Y,0,border,wht);
    rect(0,MAX_Y,MAX_X-border,MAX_X,wht);

    rect(MAX_Y/4, MAX_Y/4+3, MAX_X/2-30, MAX_X/2+30, wht);
    rect(MAX_Y/2+10, MAX_Y/2+13, MAX_X/3, MAX_X/3+60, wht);

    // initialize and display robot and player score
    int scoreH=0;
    int scoreR=0;
    displayScore(scoreH,scoreR);

    // set colours & assign structs
    Player human, robot;
    human.colour=grn; 
    robot.colour=red;
 
    // initialize game loop
    while (scoreH<MAX_SCORE && scoreR<MAX_SCORE) {
        rect(border,MAX_Y-border,border,MAX_X-border,blk);
        // initialize positions
        human.y=MAX_Y/2; human.x=MAX_X/3;   human.dir=RIGHT; human.alive=1;
        robot.y=MAX_Y/2; robot.x=2*MAX_X/3; robot.dir=LEFT;  robot.alive=1;
        // draw starting positions
        drawPixel(human.y,human.x,human.colour);
        drawPixel(robot.y,robot.x,robot.colour);

        /*while (human.alive || robot.alive) {
    
    if (tick) {
        tick = 0;  // clear event

        robotDecide(&robot);

        if (human.alive) {
            int ny=human.y, nx=human.x;
            stepPos(&ny,&nx,human.dir);
            if (!insidePlayable(ny,nx,border)||detectObstacleAhead(ny,nx))
                human.alive=0;
            else { drawPixel(ny,nx,grn); human.y=ny; human.x=nx; }
        }

        if (robot.alive) {
            int ny=robot.y, nx=robot.x;
            stepPos(&ny,&nx,robot.dir);
            if (!insidePlayable(ny,nx,border)||detectObstacleAhead(ny,nx))
                robot.alive=0;
            else { drawPixel(ny,nx,red); robot.y=ny; robot.x=nx; }
        }

        if (!human.alive && robot.alive) { scoreR++; displayScore(scoreH,scoreR); break; }
        if (!robot.alive && human.alive) { scoreH++; displayScore(scoreH,scoreR); break; }
        if (!human.alive && !robot.alive) break;
    }
}
*/


        // while they are alive, then move via outputs from the functions
        while (human.alive || robot.alive) {
            humanDecide(&human);
            robotDecide(&robot);

            if (human.alive) {
                int ny=human.y;
                int nx=human.x; 
                stepPos(&ny,&nx,human.dir);
                if (!insidePlayable(ny,nx,border)||detectObstacleAhead(ny,nx)) 
                human.alive=0;
                else { drawPixel(ny,nx,grn); human.y=ny; human.x=nx; }
            }

            if (robot.alive) {
                int ny=robot.y;
                int nx=robot.x; 
                stepPos(&ny,&nx,robot.dir);
                if (!insidePlayable(ny,nx,border)||detectObstacleAhead(ny,nx)) 
                robot.alive=0;
                else { drawPixel(ny,nx,red); robot.y=ny; robot.x=nx; }
            }

            if (!human.alive && robot.alive) {
    		scoreR++; // robot scores
   			displayScore(scoreH,scoreR);
    		break;     // end round immediately
			}
			if (!robot.alive && human.alive) {
    		scoreH++; // human scores
    		displayScore(scoreH,scoreR);
    		break;     // end round immediately
			}
			if (!human.alive && !robot.alive) break; // both dead same time, no pt awarded


            delay(speedMod());
        }

    }

    if (scoreH>=MAX_SCORE) 
        rect(0,MAX_Y,0,MAX_X,grn);

    else 
        rect(0,MAX_Y,0,MAX_X,red);

    printf("Game over. Final scores: Human=%d Robot=%d\n",scoreH,scoreR);
    return 0;
}
