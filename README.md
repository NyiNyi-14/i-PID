# Intelligent PID Control Augmented with Input Shaping for Precision Motion Control in Dynamic Systems

<p align="center">
  <b>Transformation of classical PID control loop into iPID via unknown dynamics elimination with input shaping</b><br>
  <img src="media/iPID_block.png" width="80%">
</p>

This repostitory contains the code, data, and methods used to analyze the performance of input shaping and an intelligent PID control mechanism, and compare it to that of a similar classical PID. To do this, the code simulates the behavior of these systems, graphs them, and then tabulates multiple quantitative charateristics of their trajectories to compare directly.

*Note: This project has been submitted to ACC 2026. Updates will be added as the review process progresses.*

---

## Project Structure

  - The `Libraries` folder contains custom modules for system modeling, controller design, and simulation routines used by the main scripts.
   - `run_SpringMass.py` – Simulates PID and iPID closed-loop control of a spring–mass system; generates plots and tables.  
   - `run_DC.py` – Simulates PID and iPID closed-loop control of a DC motor system; generates plots and tables.  
  - `robustness.py` – Simulates the DC motor system under noise and feedback delay using the iPID controller with input shaping to evaluate the robustness of the proposed method.

---

## Requirements

- matplotlib==3.10.3
- numpy==2.3.0
- pandas==2.3.0
- pyDOE==0.3.8
- scipy==1.15.3
- seaborn==0.13.2

Install dependencies using:

```bash
pip install -r requirements.txt
```
---

## How to Run

### Step 1: Setup

Download or clone the repository:

```bash
git clone https://github.com/NyiNyi-14/i_PID.git
```

Make sure all scripts are in the same directory.

### Step 2: Specifications

 Before running the code, update the system and controller parameters to simulate your own setup.
- System parameters: (m, k, R, L, Kb, Kt, J, B)  
- Controller tuning: (Kp, Ki, Kd)  
- Initial conditions: (x0, v0, ω, ia)  
- Simulation settings: (duration, dt)  

Set the appropriate output paths to ensure graphs are saved in the desired locations.  

### Step 3: Generate Figures and Tables

Run the main scripts to perform simulations and automatically generate figures.  

```bash
python run_SpringMass.py
```

```bash
python run_DC.py
```

- These scripts simulate dynamic systems under PID and iPID control with various reference inputs: aggressive step, exponential, normal input shaping, and robust input shaping.  

---

## Results

<p align="center">
  <b>Responses from systems to aggressive step reference.</b><br>
  <img src="media/Step_Graphs.png" width="70%"><br>
  (a) Spring-mass system  (b) DC motor system
</p>

<p align="center">
  <b>Responses from spring-mass system to normal and robust input shaped reference.</b><br>
  <img src="media/Spring_IS_Graphs.png" width="70%"><br>
  (a) PID controlled  (b) iPID controlled
</p>

<p align="center">
  <b>Quantitative evaluation of performance of spring mass with input shaped reference.</b><br>
  <img src="media/SM_Perofrmance.png" width="70%"><br>
</p>

<p align="center">
  <b>Responses from DC motor system to normal and robust input shaped reference.</b><br>
  <img src="media/DC_IS_Graphs.png" width="70%"><br>
  (a) PID controlled  (b) iPID controlled
</p>

<p align="center">
  <b>Quantitative evaluation of performance of DC motor system with input shaped reference.</b><br>
  <img src="media/DC_Performance.png" width="70%"><br>
</p>

<p align="center">
  <b>Responses from DC motor system with uncertain resistance to input shaped references.</b><br>
  <img src="media/DC_IS_Sample_Graphs.png" width="80%"><br>
  (a) PID controller with normal input shaping  (b) PID controller with robust input shaping <br>  (c) iPID controller with normal input shaping  (d) iPID controller with robust input shaping
</p>

<p align="center">
  <b>Quantitative evaluation of performance of DC motor system with uncertain resistance and input shaped reference.</b><br>
  <img src="media/DC_Sample_Performance.png" width="70%"><br>
</p>

<p align="center">
  <b>Robustness of iPID reference tracking with and without robust input shaping under a 10 ms delay</b><br>
  <img src="media/robustness.png" width="80%"><br>
  (a) σ = 0.5  (b) σ = 1  (c) σ = 2  (d) σ = 10
</p>

---

## Related Work

This project builds on developed control mechanisms, including:

- Intelligent PID

- Input Shaping

---

## Citation

If you use this work, please cite the related paper as follows:
```bash

```
<!-- 
---

## Author

**Nyi Nyi Aung** 

PhD Student, Mechanical and Industrial Engineering - LSU, USA

MSc, Sustainable Transportation and Electrical Power Systems - UniOvi, Spain

BE, Electrical Power - YTU, Myanmar

##

**Bradley Wight** 

B.S. Student, Mechanical Engineering

Louisiana State University

##

**Adrian Stein, PhD**

Assistant Professor

Department of Mechanical and Industrial Engineering

Louisiana State University

## -->