# TAS-Poseidon Automated Control

This repository contains the custom software developed to control and automate the Translaminar Autonomous System (TAS) using Poseidon syringe pumps.

## Overview

The software provides automated control of syringe pump flow rates and pressure-based feedback for the ex vivo human translaminar autonomous system. The software was developed to support automated regulation of intraocular pressure (IOP) and intracranial pressure (ICP) within the TAS model.

The code supports communication between the control computer, Arduino-based pump controllers, pressure sensors, and Poseidon syringe pumps.

## Repository Contents

* `Python/` — Python-based control and automation software
* `Arduino/` — Arduino firmware used for pump and sensor control
* `requirements.txt` — Python package requirements
* `CITATION.cff` — Citation information for this software

## Hardware

The software was developed for use with the TAS prototype and Poseidon syringe pump system described in the associated manuscript.

The system includes:

* Poseidon syringe pumps
* A4988 stepper motor drivers
* Pressure sensors
* Raspberry Pi/computer-based control
* Custom TAS hardware

## Installation and Use

See the documentation and comments within the individual software files for setup and operation instructions.

Python dependencies are listed in `requirements.txt`.

## Associated Publication

This software supports the research reported in:

**Towards High Throughput Expansion of the Ex Vivo Human Translaminar Autonomous System Using Poseidon Pump**

A DOI for the archived version of this software is provided through Zenodo.

## Version

Version 1.0 corresponds to the software version used for the experiments reported in the associated manuscript.
