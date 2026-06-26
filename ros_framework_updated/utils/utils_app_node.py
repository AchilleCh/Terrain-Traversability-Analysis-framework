#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 12:40:42 2023

@author: achille
"""



import torch
import numpy as np



    
def load_checkpoint(model,flag):
    #print('==>Loading checkpoint')
    seg_path = '/home/user/ros2_ws/src/ros_framework/ros_framework/myCheckpoint_seg_s.pth.tar'
    class_path = '/home/user/ros2_ws/src/ros_framework/ros_framework/myCheckpoint_class_rough.pth.tar'
    if flag == 'S':
        model.load_state_dict(torch.load(seg_path)['state_dict'])
    
    elif flag == 'C':
        model.load_state_dict(torch.load(class_path)['state_dict'])    
    #print('==>Checkpoint loaded')


###############################################################################

def generate_seg_cost_map(classes_predictions, predictions):

    # Cost reductions = [0.8,0.7,0.5,0.0,0.6] #reductions relative to different terrains
    # Definition as a dictionary
    reductions = {0 : 0.8,
                  1 : 0.7,
                  2 : 0.5,
                  3 : 0.0}

    cost_map = torch.ones((classes_predictions.shape[0],classes_predictions.shape[1]))

    for i, row in enumerate(classes_predictions):
        for j, el in enumerate(row):
            confidence = predictions[el,i,j]
            effective_reduction = reductions[el] * confidence
            # Reduction assignment
            cost_map[i,j] = cost_map[i,j] - effective_reduction
    
    return cost_map

################################################################################

# Inference of segmentation predicted mask and cost map:
def infer_seg_cost_map(model_seg, image, DEVICE):

    model_seg.eval()

    with torch.no_grad():
        image = image.to(DEVICE)
        y_pred_seg = model_seg(image)
        y_pred_seg = y_pred_seg.squeeze(0).to(DEVICE)
        
        
    prediction_seg = y_pred_seg.to('cpu') #probabilities
    predicted_mask = torch.argmax(prediction_seg,dim = 0) #classes prediction


    # Computation of the segmentation cost map
    prediction_seg = prediction_seg.numpy()
    predicted_mask = predicted_mask.numpy()

    cost_map = generate_seg_cost_map(classes_predictions = predicted_mask, predictions = prediction_seg)
    
    return cost_map, prediction_seg, predicted_mask

#################################################################
def generate_class_cost_map(classes_predictions, predictions):

    # Cost reductions relative to different roughness levels
    # Defintion as a dictionary
    
    reductions = {0 : 0.85,
                1 : 0.6,
                2 : 0.4,
                3 : 0.1}

    

    cost_map = torch.ones((classes_predictions.shape[0],classes_predictions.shape[1]))

    for i, row in enumerate(classes_predictions):
        for j, el in enumerate(row):
            confidence = predictions[el,i,j]
            effective_reduction = reductions[el] * confidence
            # Reduction assignment
            cost_map[i,j] = cost_map[i,j] - effective_reduction
    
    return cost_map



################################################################################

# Inference of roughness classification predicted mask and cost map:
def infer_class_cost_map(model, image, DEVICE):

    model.eval()

    with torch.no_grad():
        image = image.to(DEVICE)
        y_pred_class = model(image)
        y_pred_class = y_pred_class.squeeze(0).to(DEVICE)
        
        
    prediction_class = y_pred_class.to('cpu') #probabilities
    predicted_mask = torch.argmax(prediction_class,dim = 0) #classes prediction

    prediction_class = prediction_class.numpy()
    predicted_mask = predicted_mask.numpy()

    cost_map = generate_class_cost_map(classes_predictions = predicted_mask, predictions = prediction_class)

    return cost_map, prediction_class, predicted_mask


