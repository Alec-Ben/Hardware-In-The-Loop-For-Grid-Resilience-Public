# Hardware-in-the-Loop for Grid Resilience  
**UVM SEED Team 5 Technical Documentation (2024–2025)**

Welcome to the official GitHub repository for the UVM SEED Team 5 project: *Hardware-in-the-Loop for Grid Resilience*. This project was a collaboration between the [CREATE Research Lab](https://www.uvm.edu/cems/create) at the University of Vermont and the [Vermont Electric Power Company (VELCO)](https://www.velco.com/), carried out as part of the 2024–2025 [Senior Experience in Engineering Design (SEED)](https://www.uvm.edu/cems/me/senior-experience-engineering-design-seed) program.

## Project Overview

The goal of this project was to explore how hardware-in-the-loop (HIL) simulations can be used to support grid resilience by testing inverter controller behavior in real-time grid scenarios. Specifically, the team integrated a simulated power system case from PSS/E (via OPAL-RT’s ePHASORSIM) with a physical Arduino Mega microcontroller running active and reactive power (PQ) control logic. The simulation allowed for testing controller performance under dynamic grid conditions in a safe and repeatable environment.

By the end of the project, the team successfully:
- Configured OPAL-RT’s real-time simulation environment with ePHASORSIM.
- Developed and tested multiple RT-Lab projects incorporating PSS/E grid models.
- Programmed and connected a real Arduino-based inverter controller using custom PQ control code.
- Achieved full HIL integration with real-time signal exchange between the OPAL-RT system and the Arduino.

## Repository Structure

This repository serves as the technical documentation and codebase for the project. Below is a summary of the key contents:

### OPAL-RT User Manual  
A comprehensive guide to using the OPAL-RT hardware and software platform, including:
- Hardware setup
- Creating and developing RT-Lab projects
- Integrating PSS/E grid data using ePHASORSIM
- Running HIL simulations and collecting results

### RT-Lab Project Overview  
A walkthrough of each RT-Lab project included in the repository, explaining:
- The purpose of each project scenario
- Key model features and controller interactions
- How to run and modify the simulations

### Projects Folder  
Contains subfolders for each individual RT-Lab project. Each folder includes all necessary files to replicate the team’s simulation setup, including:

- `.m` — MATLAB script to initialize model parameters  
- `.slx` — Simulink system model for RT-Lab  
- `.mdl` — Legacy Simulink file required for RT-Lab compatibility  
- `.raw`* — Static grid data in PSS/E format  
- `.dyr`* — Dynamic data (e.g., generator parameters) in PSS/E format  
- `.xlsx`* — Spreadsheet for defining I/O points in ePHASORSIM  
- `.ino`* — Arduino code implementing PQ control logic

> \* Not all files are required for every project. File presence depends on the scope and configuration of each specific simulation.



