clear
%InitData
InitDatactrl;
%InitFunction
load MavLinkStruct;
%Initial condition
ModelInit_PosE=uav.ic.Pos_0;
ModelInit_VelB=uav.ic.Vb_0;
ModelInit_AngEuler=uav.ic.Euler_0;
ModelInit_RateB=uav.ic.PQR_0;
ModelInit_Inputs = [0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0];

ModelParam_uavType = int8(100); %这里是固定翼机型，具体定义见文档"机型定义文档.docx"
%Environment Parameter
ModelParam_envLongitude = 116.259368300000;
ModelParam_envLatitude = 40.1540302;
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];
ModelParam_envAltitude = -41.5260009765625;     %参考高度，负值


ModelParam_GlobalNoiseGainSwitch =0.4;
ModelParam_timeSampBaro = 0.01;
ModelParam_timeSampTurbWind = 0.01;
ModelParam_BusSampleRate = 0.001;
ModelParam_GPSEphFinal=0.3;
ModelParam_GPSEpvFinal=0.4;
ModelParam_GPSFix3DFix=3;
ModelParam_GPSSatsVisible=10;

%Noise Parameter
ModelParam_noisePowerAccel = [0.001,0.001,0.003];%顺序 xyz 下同  不要修改这里
ModelParam_noiseSampleTimeAccel = 0.001;
%ModelParam_noiseLowPassFilterCoeAccel = 0.0001;
ModelParam_noisePowerOffGainAccel = 0.04;
ModelParam_noisePowerOffGainAccelZ = 0.03;
ModelParam_noisePowerOnGainAccel = 0.8;
ModelParam_noisePowerOnGainAccelZ = 4.5;


ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001];%不要修改这里
ModelParam_noiseSampleTimeGyro = 0.001;
%ModelParam_noiseLowPassFilterCoeGyro = 0.0001;
ModelParam_noisePowerOffGainGyro = 0.02;
ModelParam_noisePowerOffGainGyroZ = 0.025;
ModelParam_noisePowerOnGainGyro = 2;%3.2;
ModelParam_noisePowerOnGainGyroZ = 1;



ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];%不要修改这里
ModelParam_noiseSampleTimeMag = 0.01;
%ModelParam_noiseLowPassFilterCoeMag = 0.02;%暂时没有使用
ModelParam_noisePowerOffGainMag = 0.02;
ModelParam_noisePowerOffGainMagZ = 0.035;
ModelParam_noisePowerOnGainMag = 0.025;
ModelParam_noisePowerOnGainMagZ = 0.05;



ModelParam_noisePowerIMU=0;%IMU噪声，这里是白噪声，这里是经过归一化

ModelParam_noiseUpperGPS=0.5;  %GPS定位误差噪声，均匀噪声，这里填x,y,z的波动上限，单位是m
ModelParam_noiseGPSSampTime=0.2;%默认0.05

ModelParam_noiseUpperBaro=0; %气压计噪声，均匀噪声，这里填高度的波动上限，单位是m
ModelParam_noiseBaroSampTime=0.5;%气压计噪声更新频率，，默认0.05
ModelParam_noiseBaroCoupleWithSpeed=0;%气压计测量高度与动压关系，也就是风速与气压计掉高模型的系数，当前参数0.008飞机10m/s掉高1m

ModelParam_noiseUpperWindBodyRatio=0;%风波动系数，风速*(1+该系数)
ModelParam_noiseWindSampTime=0.001;


%%ModelParam_envGravityAcc = 9.81;
ModelParam_envAirDensity = 1.225;    %还没有用到
ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar
ModelParam_noiseTs = 0.001;