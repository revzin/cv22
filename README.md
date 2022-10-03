# About me

Grisha (Grigory) Revzin, 28 y. o., Israel (oleh hadash), hardware engineer. [LinkedIn](https://www.linkedin.com/in/revzing/)

Cortex-MX MCUs, C, Python, Linux, analog and digital circuit design and PCB layout; Altium Designer & Cadence OrCAD.

Interested in hardware cost analysis and development scheduling techniques, high-performance embedded systems and real-world EMC, EMI & signal integrity work, and embedded firmware testing and verification.

# Engineering Projects

During my stay at R-SEPT (a startup-style company of 6-8 hardware, software and mechanical engineers) I've served in a dual role of a technical leader and an engineer.

Under my technical leadership we've successfully [field-tested a made-from-scratch milking robot (YT video)](https://youtube.com/watch?v=lFIApEtLz7I) with a diverse range of custom electronics tightly integrated with mechanics and pneumatics. As a leader I've distributed available tasks, reviewed team designs (mechanical, electrical & electronic and software), made decisions on system architecture and planned development and testing from budgeting and scheduling viewpoints.

As an engineer my focus was on the most difficult tasks available, ones that required technical breakthroughs. That engineering role presented diverse engineering challenges some of which are briefly described further down.

## PLC-like controllers

The milking robot ('AMS-2.1') we developed has multiple controller PCBs that all connect to a central Linux server via Ethernet (where the data transfer bandwidth is more demanding) and CAN buses (where the data transfers are limited to small infrequent updates).

To avoid dependence on PLC vendors and their often-not-so-great APIs as much as possible custom controller PCBs were planned after we worked out the number and types of I/Os in the robot by drawing up a detailed 7-page electrical wiring diagram with types and models of sensors, actuators and other equipment set. 

![PLC-like boards developed for the robot](/assets/everything.jpg)

*PLC-like boards I've either directly developed or planned and closely reviewed for the milking robot*

After getting all I/Os in a large table we distributed them between the boards grouping them by logical functions and self-sufficiency. I determined the mechanical designs of the boards by modeling the control cabinet in SolidWorks, planning primary I/O placement, 'mezzanine' I/O expansion boards and cable and wire routes.

![Control cabinet model and real-world appearance](/assets/cabinet.jpg)

*Control cabinet model and its real-world appearance*

These mechanical designs were imported into Altium and used to perform layout.

We used Wuerth's spring-loaded wire terminals in multiple rows to provide connections that are easy on the hands and don't break down after a dozen of connect-disconnect cycles.

### 'Arm' and 'Milking' controllers

Two similar boards that provide ~100 discrete I/Os for end-switches and pneumatic valves and ~20 current loop I/Os with 12- and 16-bit ADCs. All discrete outputs are capable of driving 50 mA loads at 100 Hz, some are capable of 2 A loads at 2 Hz (driving large valves). Discrete inputs are capable of 5 mA loads and are isolated from the system side. Inputs and outputs are separately protected from short circuits by trippable protector ICs (which have been proven very useful numerous times as wire insulation in long harnesses was damaged during testing). 

![Discrete output arrays](/assets/dos.png)

*Discrete output arrays*

Continuous inputs are provided by an instrumentation amplifier configured to scale 4..20 mA to ADC range with careful choice of value margins to detect current loop faults if the signal is outside the 4..20 mA range. At the logical center of the board is an STM32 MCU to which I/Os are connected over I2C, SPI and 74 series shift registers. Connection to an outside world is provided by Ethernet PHY. 

![Current loop input](/assets/c2v1.png)

*Current loop input*

The boards get commands over UDP and periodically sends back the current state in "packed" structs defined in header files shared between Linux side and board side.

The 'Arm' board features an additional STM32F3 that is limited to controlling the vertical pneumatic axis: as this is a real-time control task with the axis load violently crashing to the floor if control is removed a dedicated co-controller was a sensible choice from a safety divide-and-conquer standpoint (limiting complexity of the code in the critical control loop) even though after enough testing the main controller would've been sufficient. 

In general due to small series runs and negligible costs of our custom electronics compared to other cost centers of the robot we usually laid out PCBs for maximum EMI resistance, preferring to have more PCB ground planes and lower return current impedances than removing a PCB layer for cost savings.

!['Arm' layout fragment](/assets/arm_layout.png)

*'Arm' controller layout fragment, layout by me*

For these boards I have designed the schematics, did PCB layout for the 'Arm' one, reviewed the team's work on the other one, implemented some of the device drivers in firmware, and implemented the UDP (originally TCP) communications with the host server (FreeRTOS/LwIP).

### Conductometer 

Electrical conductivity of milk is an important indicator of the cow's health so we've added a conductivity meter into our electronics, with one channel per tit.

The circuit is designed to be used with 4-terminal sensing elements which I designed in ANSYS Maxwell. After some 'virtual' iterations we made a rough model and then a production model of the sensing element.

![ANSYS Maxwell cell electric field simulation of an earlier 6-terminal version](/assets/cond-maxwell.png)

*ANSYS Maxwell cell electric field simulation of an earlier 6-terminal version*

In 4-terminal conductivity sensing a bipolar precision square wave is put on two 'excitation' electrodes (in kHz frequency range), which creates an electric current (in our design -- limited to 10 mA) through the liquid. After passing through the liquid that current is 'sinked' into an operational amplifier configured as a current-to-voltage converter. The resistance of the liquid creates an electric potential difference between the other two 'sensing' electrodes, which is then divided by the measured current, producing the resistance of the cell, which differs from the specific conductivity of the liquid by a constant factor (the 'cell factor').

The voltage and current are measured with a double sample-and-hold (SaH) circuit. On the 'positive' part of the excitation wave the 'positive' SaH is sampling current and voltage values and the 'negative' SaH is passed on to the A/D converter. On the 'negative' part - vice versa. 

![Analog signal conditioning](/assets/cond-analog.png)

*Analog signal conditioning, current*

![Simplified measurement logic waveforms](/assets/cond-cycle.png)

*Simplified measurement control logic waveforms*

We've adapted a reference Analog Devices design to four-channel measurment (with another group of analog multiplexers enabling successive measurement, reusing the costly signal conditioning stage), an ADC frequently used by our company and an ST microcontroller. The conductometer provides 1 Hz updates with measured resistance values for each channel so the data bandwidth is minimal and therefore isolated CAN is used to deliver updates to the Linux server. 

![Conductometer PCB in detail](/assets/condmeter.jpg)

*Conductometer PCB in detail, system architecture, schematic by me*

6-layer SMT PCB with ground planes both directly under TOP and BOT layers for EMI reduction.  

![Measurement cell at the bottom of the milk receiver](/assets/condmeter_cell.jpg)

*Measurement cell at the bottom of the milk receiver*

![Housing](/assets/cell1.jpg)

*Insulated sensing element housing*

To compensate for thermal changes in conductivity four 4..20 mA input channels have been added for connection of industry standard temperature sensors.

## CAN

I developed and supported a simple CAN transport layer for communication between various low-bandwidth devices in our robots' networks. That transport layer is based on common message definitions and common API functions implemented differently on Linux (via our CAN gates) and microcontroller side (STM bxCAN).

The system underwent two iterations, first one was based on an USB-CAN-gate (schematic and PCB designed by me around an STM32F1 controller) and then as the requirements for more direct network connectivity arrived, an Ehernet-CAN-gate also designed by me. 

The system was used in the milking robot and in the wheeled robot with different CAN message sets.

## Employee Action Tracking System hardware front-end

Our company was subcontracted by a third party to develop the data acquisition front-end of an Employee Action Tracking System (EATS). Per the contract we needed to develop the means of acquiring 6-DOF movement data every day over a 9-hours shift using two wearable armbands on an electric company employee's wrists. The recorded data was then to be examined by a third-party neural network classifier that was trained to determine from this 'movement signature' if an employee was deviating from prescribed activities. 

We needed to collect accelerometer + gyroscope data at around 50 Hz. Additionally employee's heart rate was to be recorded occasionally (to check that both bracelets are on the same employee and are actually being worn). The data was to be uploaded to a third-party server once a day after the shift's end and the armbands were not to be a hindrance to employee activities.

![System architecture plan fragment showing the armband side](/assets/brc_plan.png)

*System architecture plan fragment showing the armband side*

I designed the system architecture, circuit diagrams and established PCB component placement for this project. After various considerations and a prototype I decided on a BLE-enabled armband (mainly to check the proximity of the armband to the employee's corporate phone at sparse time intervals) with a contemporary 6DOF MEMS device, a Micron QSPI Flash memory chip sized to hold a shift's worth of movement data (around 150 MByte per shift), an off-the-shelf I2C pulse-oximeter sensor, a BLE-enabled STM low-power MCU and finally USB and charging over a custom connector: gold-plated contacts flush with the armband housing of a type found on some smart watches (the armband 'clicks' into place and the USB contacts press against the base station's by magnets when the armband is placed in proximity to the slot on the base station).

![Prototype armbands charging on the base station](/assets/armband_charging_station.jpg)

*Prototype armbands charging on the base station*

After each shift the armband was to be returned a 16-armband base station which acts as a charger and a movement data access point (someone wishing to read the data will connect to the base station and request data from an armband with a given serial number 'burnt' into each armband during manufacturing). Wireless data transfer and charging were considered but ultimately rejected as they required larger enclosures to house reasonably performing antennae and 'juicer' batteries required to transfer movement data.

The armband's components' current consumption was carefully studied. Strict 'power budget' was enforced to use the smallest battery possible. The required usage of low-power modes of various components was planned and the overall current consumption in different modes over the whole shift was calculated (based on datasheets and experimental data) and verified on system level, before actual circuit design. 

After signing off on the provisional BOM I laid out a draft PCB and based on that prepared specifications to a mechanical design subcontractor, providing them with a reference STEP model of the PCB. We figured out the construction of auxiliary PCBs (a flex-PCB LED board on top of the battery and a stub to adjust the placement of the pulse-oximeter sensor). The production board was to be an eight-layer double-sided tightly packed board exceeding the domestic manufacturer's capabilities. For the purposes of firmware development we laid out a demo board identical to the production board in circuitry but with extensive testpoints, significantly 'relaxed' assembly and debug connectors.

*Customer logos obscured*
![Demo (EVT) armband board](/assets/brc_demo.jpg)

*Demo (EVT) armband board*

![Production armband board in CAD view](/assets/armband_board.png)

*Production armband board in CAD view, component placement by me*

The base station board was designed to interface with 16 armbands via a USB multiplexer and with Ethernet connectivity to enable armband data download. Base station was the USB 'host' and the armband was the 'device'. I implemented the base station firmware with FreeRTOS and LwIP, and a Python client interfacing with the base station (shared structures were described with ctypes). A connected client could request the state of each slot (if occupied, the S/N of the armband) on the base station and then download the data from a given slot. We have also fully developed the armband's firmware on the prototype board and tested every aspect of data collection and transmission.

![DVT base station board](/assets/cs_board.jpg)

*DVT base station board, layout by me*

The BLE was implemented with an SMT antenna mounted at a relative distance from conductors. Antenna placement and range were verified using small prototype antenna boards with antennas mounted at different proximity to conductors connected in place of a default antenna of a commercial Wi-Fi router (as Wi-Fi uses the same 2.4 GHz ISM band we can check antenna range with a Wi-Fi enabled smartphone, much easier then hacking an antenna to a Bluetooth-enabled device).

![Antenna range test rig](/assets/ant_test.jpg)

*Antenna range test rig*

![BLE antenna 50 Ohm controlled-impedance layout](/assets/BLE_layout.png) 

*BLE antenna 50 Ohm controlled-impedance layout*

I also designed various auxiliary helper boards, adapters and holders for handling the small and unwieldy production PCBs during flashing and debugging.

![A holding fixture and an adaptor board](/assets/adap.png) 

*A holding fixture and an adaptor board*

The project did not move beyond two EVT (engineering validation test) prototypes (first one included GNSS positioning later delegated to the company smartphone to save space).

## Real-time axis controller

The milking robot has a pneumatic cylinder-based vertical axis that is controlled by a Festo proportional valve following a 4..20 mA output controlled by an auxiliary MCU on the 'Arm' controller module. Positioning the axis is a non-trivial task, as high load variability and high stiction (due to the mechanical design of the axis) are present. I have designed and implemented a double-loop PID controller for that task. 

![Axis controller design](/assets/axis_cont.png)

*Axis controller design*

The controller uses data from two pressure sensors and two overlapping position sensors to determine the required force to move the axis into a set position.

The inner force loop adjusts the proportional valve to keep the force at setpoint. The outer position loop adjusts the force setpoint to keep the axis position at setpoint. PID scheduling is used: gains are selected based on what direction the axis must go (plant and stiction parameters are significantly different depending on if we're going up or down). 

```
void process_update(process *p, process_inputs *i)
{
	...
  /* position PID + pid mux */
	p->s.vin_pos_error = i->ven_pos_setpoint - p->o.ven_pos_actual;
	pid_update(&p->s.position_incr, &p->c.pid_param_set[COMCU_PID_POSITION_INCR],
				p->o.ven_pos_actual, p->s.vin_pos_error, i->ve_dt, p->s.vi_pid_zeroize,
				p->s.vi_settled);
	pid_update(&p->s.position_decr, &p->c.pid_param_set[COMCU_PID_POSITION_DECR],
				p->o.ven_pos_actual, p->s.vin_pos_error, i->ve_dt, p->s.vi_pid_zeroize,
				p->s.vi_settled);
	float position_pid_out = 0;
	if (p->s.vin_pos_error > 0) {
		position_pid_out = p->s.position_decr.y = p->s.position_incr.y;
	}
	else {
		position_pid_out = p->s.position_incr.y = p->s.position_decr.y;

	}
	position_pid_out = clamp(-i->ven_force_limit, position_pid_out, i->ven_force_limit);

	/* error monitor */
	p->s.vi_settled = ermo_update(&p->s.ermo, p->s.vin_pos_error, p->c.c_ermo_switch_val,
			p->c.c_ermo_settle_time, p->c.c_ermo_unsettle_time, p->o.ve_ticks, p->s.vi_disable,
			position_pid_out);
	if (p->s.vi_settled) {
		position_pid_out = p->s.ermo.pid_averager;
	}
  ...
	/* knocker */
	int knocker_enable = !p->s.vi_settled && p->o.ve_stopped;
	float knocker_out = knocker_update(&p->s.knocker, p->s.vin_pos_error, p->o.ve_ticks, knocker_enable,
										p->c.c_knock_force_incr, p->c.c_knock_force_decr,
										p->c.c_knock_force_period, p->c.c_knock_force_duration, p->s.vi_disable);

	p->s.vin_force_setpoint = position_pid_out + knocker_out;
	if (p->c.c_force_sp_ovr) {
		p->s.vin_force_setpoint = p->c.c_force_sp;
	}
	/* force PID */
	p->s.vin_force_error = p->s.vin_force_setpoint - p->o.ven_force_actual;
	pid_update(&p->s.force, &p->c.pid_param_set[COMCU_PID_FORCE],
			   p->o.ven_force_actual, p->s.vin_force_error, i->ve_dt, p->s.vi_pid_zeroize, 0);
	float current_out = p->c.c_current_flip ? -p->s.force.y : p->s.force.y;
	current_out = filt_ema_update(&p->s.current_ema, current_out, p->c.c_ema_current_order);
	float current_upscaled = 12.0 + 8.0 * current_out;
	...
}
```

*Axis controller C implementation fragment (position and force loop update)*

To compensate for high stiction a 'stuck' state detector was designed that adds a 'knocker' signal to the force setpoint, as if attempting to 'knock' the stuck axis with a small hammer. 

![Stiction compensator at work](/assets/stiction-comp.jpg) 

*Stiction compensator at work*

To make informed decisions on the parameters and design of the controller I've coded a Python tool to edit controller constants and view real-time controller parameters and plot them. The primary MCU on the board sends UDP datagrams describing the system state obtained from the auxiliary MCU, and the Python tool records them for further plotting and sends tuning values set by the user. Each non-editable parameter can be plotted (matplotlib). 

![Tuning UI fragment](/assets/tuning_ui.jpg) 

*Tuning UI fragment showing PID controller gains tuning*

![Position following at work](/assets/pos_follow.jpg) 

*Position following at work*

[Robot arm movements showing the vertical axis at work (YT video)](https://youtu.be/mvV3rfTB1-k)

## Robot manipulator model

Filling in for an absent programmer on a time-critical task I've implemented a geometrical model of the robot's manipulator providing forward and inverse kinematics, synchronization with the 'hardware' robot, collision detection and visualization.

The model in action is seen [here (YT video)](https://youtu.be/mvV3rfTB1-k).

![Model bounding boxes for collision checks](/assets/bbox.jpg) 

*Model bounding boxes for collision check*

![The model is used to calculate world positions of detected tits](/assets/model_tits.jpg) 

*The model is used to calculate world positions of detected tits*

## BLDC motor controller

We had a side project which was based on a medium-size battery-powered wheeled robot. After quickly becoming dissatisfied with off-the-shelf BLDC motor controllers (either with low engineering quality or complicated 'black box' integration) we decided to develop our own three-phase inverter controller (60V 60A nominal).

I was responsible for providing general specifications of the module, established the primary performance parameters, educated the team on motor control topics (I2t management, high-side MOSFET driving methods, overcurrent protection), was responsible for schematic review and BOM, and coordinated mechanical integration with the mechanical team. I performed PCB layout and coded MCU firmware implementing a simple cascade-PID controller (current - velocity  - position loops). 

![DVT stage board](/assets/unidriver.png) 

*The latest iteration of our in-house BLDC controller. Schematics, firmware architecture and component placement by me.*

Due to the dangers associated with the actual three-phase inverter switching (high risk of fiery destruction of either the board or the motor in case of switching sequence violations) I decided to offload the commutation logic to an entry-level Intel (Altera) FPGA containing switching BLDC controller I've implemented in Verilog. That controller performed the switching sequence by handling the motor's Hall sensors, followed PWM commands from the MCU, and reacted to fast and slow overcurrent signals from the current sensor. All external signals are synchronized to internal FPGA clocks and digitally debounced. 

```
always @(posedge CLK or negedge MOTOR_EN)
		if (!MOTOR_EN) begin
			U <= Z;
			V <= Z;
			W <= Z;
			MOTOR_HALL_INVALID <= 1'b0;
		end else if (motor_ovcu) begin
			U <= Z;
			V <= Z;
			W <= Z;
			MOTOR_HALL_INVALID <= 1'b0;
		end else if (BRAKE) begin
			U = L_REG;
			V = L_REG;
			W = L_REG;			
			MOTOR_HALL_INVALID <= 1'b0;
		end else case (HALL)
			3'b001 : begin
				W <= NH;
				V <= NL;
				U <= Z;
				MOTOR_HALL_INVALID <= 0; end
			3'b101 : begin
				U <= NH;
				V <= NL;
				W <= Z;
				MOTOR_HALL_INVALID <= 0; end
        ...
			default : begin
				U <= Z;
				V <= Z;
				W <= Z;
				MOTOR_HALL_INVALID <= 1; end
		endcase
```
*Fragment of Verilog code in the FPGA handling inverter phasing*

Various iterations of our controller drove [BLDC motors up to 1 kW (YT video of our Feed Pusher prototype)](https://youtube.com/watch?v=exd6W7MBLxs).
