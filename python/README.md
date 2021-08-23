# IMU Calibration and Online Self-correction

The code contained in this folder was written as a part of the **Google Summer of Code 2021** program ([GSoC Project Page](https://summerofcode.withgoogle.com/projects/#4925832792899584))

This folder contains code for performing continuous online calibration of an Inertial Measurement Unit (IMU) using Recursive Least Squares ([RLS](https://en.wikipedia.org/wiki/Recursive_least_squares_filter)) and offline batch calibration of an IMU using Dorveaux's iterative least squares based method. The folder also includes code for performing attitude estimation by fusion of accelerometer and magnetometer data using filters from Attitude & Heading Registration System ([AHRS](https://ahrs.readthedocs.io/en/latest/index.html)).
Tools for testing single-stage calibration/2-stage calibration, plotting the results, saving results and tuning parameters have also been included.

---

### Software Dependencies

* Python 3.6.9
* Numpy 1.19.5
* SciPy 1.5.1
* Pandas 1.0.5
* Matplotlib 3.3.4
* AHRS Python
* PyYAML 3.12
* Sys
* Pymavlink
* CSV

### Installation

```
git clone https://github.com/uccross/open-source-autonomous-vehicle-controller/
cd open-source-autonomous-vehicle-controller/python
```
---
## Usage

### Testing offline calibration

The raw data of a single sensor is stored in a csv file with three columns (one for each axis).

Use the following command from the `python` folder to get calibration parameters using Dorveaux:
```
python3 dorveaux.py <name_of_csv_file>
```
If the parameters vary over time, use the Dorveaux fixed-lag method:
```
python3 dorveaux_fixed_lag.py <name_of_csv_file>
```

### Testing RLS

Recursive least squares calibration can be done with or without an initial batch. For testing RLS for a single sensor set the parameter `initial_batch_size` to the requried size in `python/recursive_least_squares.py`. 
Setting the `use_batch_only` parameter to **True** will perform only least squares based calibration of the initial batch without proceeding to RLS.

Next, in the `main()` function, set the `normalizer` to the desired magnitude of the sensor data vector (e.g. set the normalizer to 9.8 for accelerometer).

To test RLS on saved data, the format of the csv file must be the same as that for offline calibration. Run the following command to test recursive least squares:

```
python3 recursive_least_squares.py <path_to_sensor_data>
```
**TODO:** Handle NaN values while calcuating MSE

### Testing full online calibration

The parameters for full online calibration are stored in the file `config.yaml`. The steps to simulate a continuous sensor data stream and perform RLS-based online calibration are as follows:

1. In `config.yaml`, set the values of the physical constants (gravity in m/s^2 and magnetic field in Gauss).
1. Set the value of lambda (0.99 reccomended)
1. If tolerance-based screening is to be used, set `use_tolerance` to **True** and set the accelerometer and magnetometer % tolerances.
1. The `get_next_meas()` function in the `test_utils.py` file simulates a continuous data stream by reading from a CSV file containing data from all sensors. The ranges and the scale in this function need to be set according to the format of the CSV file so that the function returns an array containing the rescaled acc, gyro & mag values.

Run the following command to test full online calibration:

```
python3 calib_all_sensors.py <path_to_logfile>
```
**Note:** Since there is no reference to the true orientation available, it is assumed that the gyroscope uses only initial batch calibration. Hence, the gyroscope errors are currently not evaluated here.

### 2-stage calibration

In two stage calibration, the microcontroller pre-calibrates the raw data using the initial batch parameters. RLS then works on this pre-calibrated data to improve calibration. This is simulated using the `get_next_meas_calib()` function in `test_utils.py`, which takes in batch parameters and the logfile as input and returns an array containing pre-calibrated measurements from acc, gyro and mag.

To test 2-stage full online calibration, perform the following steps in addition to the ones listed in the previous subsection.

1. In `config.yaml`, set `use_initial_batch` to **True** 
1. Set the paths for `acc_file`, `mag_file` and `gyro_file` relative to the `/python` folder. Make sure that each file contains a 4x4 transformation matrix in a numpy-readable format i.e. rows separated by newline and values separated by space.

Use the same command as before
```
python3 calib_all_sensors.py <path_to_logfile>
```
### AHRS
The AHRS sensor fusion algorithms combine the measurements from the three sensors of the IMU to estimate orientation. The file `filters.py` in this repository contains an implementation of a framework to test and compare these algorithms. Currently, the framework contains static filters (i.e. without a dynamic model of a vehicle) that use accelerometer and magnetometer data only (i.e. not gyroscope) to estimate an orientation.

Three tests have been implemented with other helper functions for analysing the results. The tests can be called from the main function with command line arguments. Each test is described in the respective docstring.

The tunable parameters for each algorithm are included in the file `ahrs_params.yaml`.

E.g. for test #3:

```
python3 filters.py <acc_csv> <mag_csv> ahrs_params.yaml <filter_name>
```
---
## Files/Directories Included
Following is a short description of the files included and their contents.

### utils.py 
Contains all the common classes and utility functions required for the code. The `Imu` class is used for a single 9-axis IMU measurement, along with functions to normalize, vectorize and calibrate using supplied parameters.
The `CalibParams` class contains the matrices **A** (rotation/skew and scales) and **B** (Bias). It can be initialized through implicit parameters, from a file and directly. It also contains functions to correct raw measurements and save parameters.

### recursive_least_squares.py

Contains the `RecursiveLeastSquares` class. This class stores the RLS parameters and implicit weights for a single sensor. It also contains functions to convert measurements to the implicit ellipsoid form, to perform RLS iterations and to restore previous weights in case of complex scales or invalid weights.
The function `recover_params()` converts implicit weights into the matrix form (A,B) and `plot_errors()` plots the error histogram and the MSE over time.

The `main()` function takes a single sensor logfile as a parameter and performs an initial batch calibration, followed by RLS for the remaining data.


### test_utils.py

This includes utilities required to test calibration. The `send_next_meas()` and `send_next_meas_calib()` functions read a logfile and supply IMU raw/ precalibrated measurements for RLS calibration. The calibration code calls one of these functions wach time it is ready for a new measurement. While working with a real system, this function can directly be replaced with a function that reads and processes a new mavlink message.

### config.yaml
This file contains all configuration parameters needed for RLS, outlier screening, 2-stage calibration, mavlink, etc.

### plot_stats.py

The `Stats` class stores raw and calibrated measurements and computes statistics such as MSE, standard deviation, etc. The `append()` function adds a new set of measurements and updates the stats accordingly, while the `plot_rt()` function updates a pyplot with the recent measurements and stats. The plot is currently updated after every `num_vals`(default: 40) measurements.

**TODO:** Replace matplotlib.pyplot functions with a faster animation library to improve speed.


### calib_all_sensors.py
This code combines all the previous codes to perform online single stage or 2-stage calibration and plot the error stats in real time. The gyroscope online recalibration code has been commented out and can be used once AHRS (or any other filter) provides an accurate orientation estimate. Further, skeleton code has been inculded to easily incorporate kinematic/ dynamic models of the vehicles, whenever ready.

### dorveaux.py
Performs batch calibration on normalized measurements of a single 3-axis sensor using Dorveaux's iterative least squares based ellipsoid fitting and plots the MSE, histograms and 3D points (calibrated & raw).

### dorveaux_fixed_lag.py
Uses Dorveaux's method to estimate calibration parameters of successively recieved data by using only a fixed number of most-recent values, so that changing IMU parameters can be dealt with.

### filters.py
Uses nine static filters from the [ahrs](https://github.com/Mayitzin/ahrs) library in the `static_filters` dictionary. 
The `estimate_orientation()` function takes a filter name as an input, along with single measurements from the accelerometer and gyroscope to estimate an orientation using default parameters. The filter names are according to the keys used in the `static_filters` dictionary. Output can be obtained in the angular form as well as quaternion form. The `get_delta_theta()` function calculates the magnitude of difference between consecutive angles, given an array of XYZ Euler angles.

Three tests have been implemented currently.

**Test 1** calculates a pair of orientations given a filter name and 2 pairs of accelerometer-magnetometer measurements.

**Test2** takes acc and mag data from CSV files, parameters from a YAML file and a filter name as a command line argument to give an array of orientations in the form of Euler angles.

**Test3** compares orientation estimates from all working static filters to that from a given filter (passed as an argument) in degrees error about each axis.

The main function calls one or more of these tests.

### ahrs_params.yaml
This file contains the tunable parameters of each implemented static filter and is read by filters.py.

### tests/
This folder contains simulated and real datasets used for the tests.

### batch_params/
This folder contains the stored/ default batch parameters for 2-stage calibration in numpy readable format. They are stored in the form of a 4x4 3D transformation matrix. The default matrix as an identity matrix.

## Future Work

* **Working with Mavlink data from a microcontroller**

* **Speeding up realtime plotting**

* **Including kinematic/dynamic model of the vehicle**

* **More AHRS filters** (Dynamic filters and gyro-based filters)

---
## More Info
The theoretical overview, mathematical derivations and the results of the tests are included in the [Final Report](https://drive.google.com/file/d/11UWRHX66BA1PApXeGKayeBiXhyOOrC5R/view)

---
## Contributers:

* **Rishikesh Vanarse** *([Github](https://github.com/rmvanarse), [Website](https://rmvanarse.github.io))*, Computer Science, BITS Pilani, Goa
* **Aaron Hunter** *([Github](https://github.com/2ahunter))*
* **Pavlo Vlastos** *([Github](https://github.com/PavloGV))*

---
