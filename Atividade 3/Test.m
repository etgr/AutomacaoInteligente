clc, clear, close all;

goalSeekController = readfis('GoalSeeking.fis');

out = evalfis([0 0], goalSeekController);