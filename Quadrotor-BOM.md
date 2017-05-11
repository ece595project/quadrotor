---
title: Quadrotor BOM
layout: default
description: Quadrotor SAR project quadrotor bill of materials.
---

# {{page.title}}

{{page.description}}

[Home](https://ece595project.github.io/quadrotor/)

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Bill of Materials

We decided that the best path forward with our quadrotors would be to build our quadrotors to spec rather than buying them as a kit. Buying the quadrotors as a kit would be cost-prohibitive, as a quadrotor kit with vision and sensor capabilities cost upwards of several hundred dollars.

One quadrotor that we spec'ed out was the DJI Phantom, which included our base requirements of having GPS and vision capabilities.

However, according to [DJI's site](http://store.dji.com), the cost of a quadrotor with these capabilities was roughly $330.

By building the quadrotors ourselves, we would be able to reduce the cost per quadcoptor by at least $100 per unit. To achieve this and to maintain interoperability with ROS, which was a requirement of our project, we would need to source our parts from multiple vendors. The two vendors we chose were Adafruit, where we were able to source the Raspberry Pi Zeros and accessories, and eBay, where we were able to source low-cost modules and sensors.

## Vendors


### Adafruit
| Part Number	| Description	| Weight (g)	| QTY	| Cost/ea.	|
| :--- | :--- | :---: | :---: | :---: |
|2885 | Raspberry Pi Zero |9.00 g |	1	| 5	|
|3157 | Raspberry Pi Zero Camera Cable	 | -- |	1 |	5.95	|
|3099 | Raspberry Pi Camera Board v2 - 8 Megapixels | -- |		1 |	29.95	|
|3196 | Pimoroni Zero LiPo | 2.60 g | 1 |	1	9.95	|
|2471 | ESP8266 Breakout (WiFi) |	--	| 0 |	9.95	|
|3070 | RFM69HCW Breakout (Radio) |	--	| 1 |	9.95	|
|1781 | Lithium Ion Cylindrical Battery - 3.7v 2200mAh |	46.00 g |	1	| 9.95	|

### eBay


| Part Number	| Description	| QTY	| Cost/ea.	|
| :--- | :--- | :---: | :---: | :---: |
| NEO-M8N	NEO-M8N | GPS Module	 |	1	 | 13.02 |
| MPU 9250 |	Gyro + Accelerometer + Magnetometer |		1 | 	3.16 |
| | Electronics for Power and flight controller |		1	| 29.99 |
| RFM69HCW |	RFM69HCW (Radio) |		1 | 	4.25 |
| X5C |	2 Pairs DC Motor |		1 | 	29.99 |
| X5C |	Propellers |		2 | 	0.99 |


Total
$143.19

Total Weight
157.60 g
