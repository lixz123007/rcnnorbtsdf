import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import string
import os
from argparse import ArgumentParser
from mmdet.apis import inference_detector, init_detector, show_result_pyplot
import torch
import cv2
import numpy as np
import mmcv
import time
import os.path
import shutil

if not os.path.exists('associations.txt'):
    print("associations.txt does not exist")
    exit(0)

save_path="/home/finch/data/scnet_dl_data_walk_hsp/"
# if os.path.exists(save_path):
#     shutil.rmtree(save_path)
# os.makedirs(save_path)
# os.makedirs(save_path+"depth")
# os.makedirs(save_path+"rgb")

# initialize
print(sys.version)
print('torch -V: ', torch.__version__)
config = '/home/finch/mmdetection/configs/scnet/scnet_r50_fpn_1x_coco.py'
# Setup a checkpoint file to load
checkpoint = '/home/finch/mmdetection/checkpoints/scnet_r50_fpn_1x_coco-c3f09857.pth'
# initialize the detector
model = init_detector(config, checkpoint, device='cuda:0')
ii=0
with open("asso_walk_hsp.txt","r") as ass:
    for ass_data in ass:
        if len(ass_data)<50 :
            break
        common_name="/home/finch/data/rgbd_dataset_freiburg3_walking_halfsphere/"

        rgb_input_name=common_name+ass_data[18:43]
        depth_input_name=common_name+ass_data[62:89]

        result = inference_detector(model, rgb_input_name)

        img = cv2.imread(rgb_input_name)


        if isinstance(result, tuple):
            bbox_result, segm_result = result
            if isinstance(segm_result, tuple):
                segm_result = segm_result[0]  # ms rcnn
        else:
            bbox_result, segm_result = result, None
        bboxes = np.vstack(bbox_result)
        labels = [
            np.full(bbox.shape[0], i, dtype=np.int32)
            for i, bbox in enumerate(bbox_result)
        ]
        labels = np.concatenate(labels)


        adepth = cv2.imread(depth_input_name,-1)
        img3=np.zeros((img.shape), np.uint8)
        img4=np.zeros((adepth.shape), np.uint16)
        #img4=255

        if segm_result is not None and len(labels) > 0:  # non empty
            segms = mmcv.concat_list(segm_result)
            inds = np.where(bboxes[:, -1] > 0.7)[0]#!!!jingzhundu

            for i in inds:
                i = int(i)
                if labels[i]!=0:
                    continue;
                sg = segms[i]
                if isinstance(sg, torch.Tensor):
                    sg = sg.detach().cpu().numpy()
                mask = sg.astype(bool)
                img[mask]=255
                adepth[mask]=0
                
                #img4[mask]=0
                img3[mask]=255


        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(img3,kernel,iterations = 1)
        dilation = cv2.dilate(dilation,kernel,iterations = 1)
        dilation = cv2.dilate(dilation,kernel,iterations = 1)
        
        kernel = np.ones((3,3),np.uint8)
        dilation = cv2.dilate(dilation,kernel,iterations = 1)
        kernel = np.ones((1,1),np.uint8)
        dilation = cv2.dilate(dilation,kernel,iterations = 1)
        dilation2 = cv2.cvtColor(dilation,cv2.COLOR_BGR2GRAY)
        # mask_inv=np.zeros((adepth.shape), np.uint16)
        #erosion = cv2.erode(img3,kernel,iterations = 1)
        mask_inv = cv2.bitwise_not(dilation2)
        #mask_inv = cv2.cvtColor(mask_inv,cv2.COLOR_BGR2GRAY)
        # cv2.imshow('dilation2', dilation2)
        # cv2.imshow('mask_inv', mask_inv)
        # cv2.waitKey(0)
        
        mask_inv = np.uint16(mask_inv)
        # print(mask_inv.dtype)
        # print(mask_inv.shape)
        # print(adepth.dtype)
        #mask_inv = np.float32(mask_inv)
        rgb_di=np.zeros((img.shape), np.uint8)
        rgb_di=cv2.add(img,dilation)#,rgb_di,dilation2)
        #cv2.add(adepth,mask_inv,img4,dilation2)

        cv2.imwrite(save_path+'rgb/'+ass_data[22:43],rgb_di)
        #cv2.imwrite(save_path+'depth/'+ass_data[68:89],img4)
        #cv2.imwrite(save_path+'rgb/'+ass_data[22:43],rgb_di)
        #cv2.imwrite(save_path+'depth/'+ass_data[68:89], adepth)
        #cv2.imwrite(save_path+'rgb/'+ass_data[22:43], img)
        print(ii)
        ii=ii+1
        # if(ii>10):
        #     break
exit(0)
