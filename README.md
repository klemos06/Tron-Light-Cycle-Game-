# Tron Light Cycle 

Tron Light Cycle is a fast-paced arcade-style game inspired by the classic Tron universe. Players control a "light cycle" that leaves a trail behind, requiring  strategic movement to avoid collisions with an outer barrier or a trail. 

The project focuses on interfacing a DE-10 Lite FPGA to function as user inputs via switches and push buttons, while using VGA to print images onto the screen.

# Features

**Gameplay**

- Players leave a growing trail behind them with their corresponding colours
- Crashing into walls, trails, or opponents results in a death
- Score is displayed and continuously updated following every round on FPGA seven segement display
- Winning player's colour displayed following 9 rounds of gameplay
- Prints to terminal final score and initial welcome message.

**Smooth & Responsive Controls**
- Debounced buttons providing precise inputs for accurate movements
- 10 adjustablle switches each uniquely corresponding to a different game speed for beginner to advanced gameplay

**Dynamic Game Loop**

- Continuous input polling and update-render cycle through VGA
- Collision detection for walls, player trails, and opponents 
- Updates player trails and positions readily with over 200 updates per second, but adjustable to desired difficulty

**Visual Design**
- Retro visuals with neon aesthetic and black background with white border mimicking old arcade games
- High contrast visuals for easy gameplay readability
- Obstacles to add depth to field of play

**Single Player**
- Play against AI programmed to think 2 steps ahead for a challenging and ever-changing gameplay 

# Demo

(Replace with real images or GIFs)

Gameplay Screenshot

Game Over Screen

Start Menu

# Tech Stack

**Language**
- C — bare-metal programming for real-time game logic, rendering, and hardware control

**Hardware & I/O**

- FPGA (DE10-Lite)
    - Memory-mapped VGA framebuffer for pixel-level graphics output
    - Physical push-buttons used as real-time player input via memory-mapped I/O
    - Physical switches used as modifiable speed inputs via memory-mapped I/O
    - 7-segment displays for live score tracking

# AI Opponent 
- Features a rule-based AI that evaluates future positions every frame to avoid collisions
- Checks forward, left, and right positions using real-time reads of the frame buffer

# Why I Built This

I built the Tron Light Cycle Game to strengthen my understanding of:

- Using memory-mapped I/O and continuous input polling

- Collision detection and non-playable AI logic 

- Pixel display using frame buffers and VGA

- Interfacing bare-metal C code with an FPGA to provide input and display output

Overall, this project helped bridge the gap between my understanding of FPGAs as a tool for digital logic using SystemVerilog and creating video games in C - teaching me that FPGAs can be used as a piece of hardware to connect C code to a user for practical purposes.

# Future Improvements

- AI difficulty levels

- Multiplayer support

- Randomly generated obstacles unique to each round

- Sound effects and background music

# How to Run Locally

**You will need the NIOS V Command Shell (Quartus Prime 24.1std) dowloaded prior to the following steps**

Clone repository:

git clone https://github.com/your-username/tron-light-cycle.git

cd tron-light-cycle

Open **NIOS V Command Shell** and type the following when you are situated in the correct repository e.g C:\Users\Bob\Downloads\cloned-repository-location:

1. make DE-10 Lite (if you are using a DE1-SoC, use make DE1-SoC)
2. make GDB_SERVER

Open a new **NIOS V Command Shell** window and type the following:

3. make TERMINAL

Open another **NIOS V Command Shell** window and type the following:

4. make COMPILE 
5. make RUN 

# Contact

If you're a recruiter, developer, or game enthusiast interested in this project, feel free to reach out via LinkedIn or email (listed on my GitHub profile).
