#pragma once

const int N_SAMPLES = 20;
const int MIN_PATH_SIZE = 25;
const int HORIZON = 4;

const double TRACK_LENGTH = 6945.554; // in meters

const double EACH_POINT_TIME = 0.02;

const double VELOCITY_INCREMENT_LIMIT = 0.10; //--.12

const double SPEED_LIMIT = (50.0 / 2.24) - 1.2; // in meter/sec -- 1.2

const double FOLLOW_DISTANCE = 7.0; // in meters

const double VEHICLE_RADIUS = 1.35; // in meters -- 1.35

const double TIME_DIFF = 0.2;

//Cost Functions weights
const double COLLISION_COST_WEIGHT = 99999;
const double BUFFER_COST_WEIGHT = 50;
const double IN_LANE_BUFFER_COST_WEIGHT = 1000;
const double EFFICIENCY_COST_WEIGHT = 10000;
const double D_DIFF_COST_WEIGHT = 500;
const double RIGHT_LANE_COST_WEIGHT = 100;