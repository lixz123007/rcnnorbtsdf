import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from argparse import ArgumentParser
from mmdet.apis import inference_detector, init_detector, show_result_pyplot
import torch
import cv2
import numpy as np
import mmcv
import time
import os.path
import os
import shutil
def printHello():
    print(sys.version)
    print('torch -V: ', torch.__version__)
    print("hello world!")


def masktry():
    t0=time.time();
    print(sys.version)
    #print('torch -V: ', torch.__version__)
    config = '/home/finch/SLAM/src/orbtsdf-openmm/mmdetection/configs/faster_rcnn/faster_rcnn_r50_fpn_1x_coco.py'
    # Setup a checkpoint file to load
    checkpoint = '/home/finch/SLAM/src/orbtsdf-openmm/mmdetection/checkpoints/faster_rcnn_r50_fpn_1x_coco_20200130-047c8118.pth'
    # initialize the detector
    config = '/home/finch/SLAM/src/orbtsdf-openmm/mmdetection/configs/mask_rcnn/mask_rcnn_r50_fpn_1x_coco.py'
    # Setup a checkpoint file to load
    checkpoint = '/home/finch/SLAM/src/orbtsdf-openmm/mmdetection/checkpoints/mask_rcnn_r50_fpn_1x_coco_20200205-d4b0c5d6.pth'
    # initialize the detector
    config = '/home/finch/SLAM/src/mmdetection/configs/scnet/scnet_r50_fpn_20e_coco.py'
    checkpoint = '/home/finch/SLAM/src/mmdetection/checkpoints/scnet_r50_fpn_20e_coco-a569f645.pth'
    
    
    model = init_detector(config, checkpoint, device='cuda:0')

    t1=time.time()

    sum=0
    for j in range(10000):
        t1=time.time()
        print("start i:",j)
        depth = '/home/finch/data/depth/'+str(j+1)+'.png'
        imgad = '/home/finch/data/rgb/'+str(j+1)+'.jpg'
        
        while(not(os.path.isfile(depth))):
            time.sleep(0.05)

        result = inference_detector(model, imgad)
        t2=time.time()
        
        print(j,t2-t1)
        img = cv2.imread(imgad)
        #ming!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        #imgre=img.copy()
        #img = img.copy()
        #img3=np.zeros((img.shape), np.uint8)
        t3=time.time()
        # t1=time.time()
        #print(result)
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
        
        adepth = cv2.imread(depth,-1)
        if segm_result is not None and len(labels) > 0:  # non empty
            segms = mmcv.concat_list(segm_result)
            inds = np.where(bboxes[:, -1] > 0.7)[0]#!!!jingzhundu
            np.random.seed(42)
            for i in inds:
                i = int(i)
                #if labels[i]!=0 and labels[i]!=15:
                if labels[i]!=0:
                    continue;

                sg = segms[i]
                if isinstance(sg, torch.Tensor):
                    sg = sg.detach().cpu().numpy()
                mask = sg.astype(bool)
                #img3[mask]=255
                img[mask]=255
                adepth[mask]=0
                #img[mask] = img[mask] * 0.5 + color_mask * 0.5


        # t4=time.time()
        cv2.imwrite('/home/finch/data/adepth/'+str(j+1)+'.png',adepth)
        cv2.imwrite('/home/finch/data/argb/'+str(j+1)+'.jpg',img)
        t5=time.time()
        ntime=float(str(t5-t1))
        sum=sum+float(str(t5-t1))
        print("pythontime:"+str(ntime))
        print("avg: "+str(sum/(1+j)))

    return res#print(result)
    # show the results
    #show_result_pyplot(model, img, result, score_thr=0.3)
    #img2 = model.show_result(img, result, score_thr=0.8, show=False)
    #cv2.imwrite('yzma2.jpg',img2)

# shutil.rmtree(r"/home/finch/data")
# os.makedirs("/home/finch/data") 
# os.makedirs("/home/finch/data/depth")
# os.makedirs("/home/finch/data/rgb")
# os.makedirs("/home/finch/data/adepth")
# os.makedirs("/home/finch/data/argb")
masktry()
