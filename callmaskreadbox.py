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


if not os.path.exists('associations.txt'):
    print("associations.txt does not exist")
    exit(0)

save_path="/home/finch/data/gfl_data_sit_rpy/"
if not os.path.exists(save_path):
    os.makedirs(save_path)
    os.makedirs(save_path+"depth")
    os.makedirs(save_path+"rgb")
# initialize
print(sys.version)
print('torch -V: ', torch.__version__)
config = '/home/finch/mmdetection/configs/scnet/scnet_r50_fpn_1x_coco.py'
# Setup a checkpoint file to load
checkpoint = '/home/finch/mmdetection/checkpoints/scnet_r50_fpn_1x_coco-c3f09857.pth'
# initialize the detector
model = init_detector(config, checkpoint, device='cuda:0')
ii=0
with open("asso_sitting_rpy.txt","r") as ass:
    for ass_data in ass:
        if len(ass_data)<50 :
            break
        common_name="/home/finch/data/rgbd_dataset_freiburg3_sitting_rpy/"

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
        score_thr=0.7

        if score_thr > 0:
            assert bboxes.shape[1] == 5
            scores = bboxes[:, -1]
            inds = scores > score_thr
            bboxes = bboxes[inds, :]
            labels = labels[inds]

        for i, (bbox, label) in enumerate(zip(bboxes, labels)):
            if labels[i] != 0:
                continue
            bbox_int = bbox.astype(np.int32)
            first_point=(bbox_int[0], bbox_int[1])
            last_point=(bbox_int[2], bbox_int[3])
            
            cv2.rectangle(img, first_point, last_point, (255, 255, 255), -1)
            cv2.rectangle(adepth, first_point, last_point, (0, 0, 0), -1)
            


        # if segm_result is not None and len(labels) > 0:  # non empty
        #     segms = mmcv.concat_list(segm_result)
        #     inds = np.where(bboxes[:, -1] > 0.7)[0]#!!!jingzhundu

        #     for i in inds:
        #         i = int(i)
        #         if labels[i]!=0:
        #             continue;
        #         sg = segms[i]
        #         if isinstance(sg, torch.Tensor):
        #             sg = sg.detach().cpu().numpy()
        #         mask = sg.astype(bool)
        #         img[mask]=255
        #         adepth[mask]=0


        cv2.imwrite(save_path+'depth/'+ass_data[68:89], adepth)
        cv2.imwrite(save_path+'rgb/'+ass_data[22:43], img)

        print(ii)
        ii=ii+1
exit(0)

def showbboxes(img,
                      bboxes,
                      labels,
                      segms=None,
                      class_names=None,
                      score_thr=0,
                      bbox_color='green',
                      text_color='green',
                      mask_color=None,
                      thickness=2,
                      font_size=13,
                      win_name='',
                      show=True,
                      wait_time=0,
                      out_file=None):
    """Draw bboxes and class labels (with scores) on an image.

    Args:
        img (str or ndarray): The image to be displayed.
        bboxes (ndarray): Bounding boxes (with scores), shaped (n, 4) or
            (n, 5).
        labels (ndarray): Labels of bboxes.
        segms (ndarray or None): Masks, shaped (n,h,w) or None
        class_names (list[str]): Names of each classes.
        score_thr (float): Minimum score of bboxes to be shown.  Default: 0
        bbox_color (str or tuple(int) or :obj:`Color`):Color of bbox lines.
           The tuple of color should be in BGR order. Default: 'green'
        text_color (str or tuple(int) or :obj:`Color`):Color of texts.
           The tuple of color should be in BGR order. Default: 'green'
        mask_color (str or tuple(int) or :obj:`Color`, optional):
           Color of masks. The tuple of color should be in BGR order.
           Default: None
        thickness (int): Thickness of lines. Default: 2
        font_size (int): Font size of texts. Default: 13
        show (bool): Whether to show the image. Default: True
        win_name (str): The window name. Default: ''
        wait_time (float): Value of waitKey param. Default: 0.
        out_file (str, optional): The filename to write the image.
            Default: None

    Returns:
        ndarray: The image with bboxes drawn on it.
    """
    assert bboxes.ndim == 2, \
        f' bboxes ndim should be 2, but its ndim is {bboxes.ndim}.'
    assert labels.ndim == 1, \
        f' labels ndim should be 1, but its ndim is {labels.ndim}.'
    assert bboxes.shape[0] == labels.shape[0], \
        'bboxes.shape[0] and labels.shape[0] should have the same length.'
    assert bboxes.shape[1] == 4 or bboxes.shape[1] == 5, \
        f' bboxes.shape[1] should be 4 or 5, but its {bboxes.shape[1]}.'
    img = mmcv.imread(img).astype(np.uint8)

    if score_thr > 0:
        assert bboxes.shape[1] == 5
        scores = bboxes[:, -1]
        inds = scores > score_thr
        bboxes = bboxes[inds, :]
        labels = labels[inds]
        if segms is not None:
            segms = segms[inds, ...]

    mask_colors = []
    if labels.shape[0] > 0:
        if mask_color is None:
            # random color
            np.random.seed(42)
            mask_colors = [
                np.random.randint(0, 256, (1, 3), dtype=np.uint8)
                for _ in range(max(labels) + 1)
            ]
        else:
            # specify  color
            mask_colors = [
                np.array(mmcv.color_val(mask_color)[::-1], dtype=np.uint8)
            ] * (
                max(labels) + 1)

    bbox_color = color_val_matplotlib(bbox_color)
    text_color = color_val_matplotlib(text_color)

    img = mmcv.bgr2rgb(img)
    width, height = img.shape[1], img.shape[0]
    img = np.ascontiguousarray(img)

    fig = plt.figure(win_name, frameon=False)
    plt.title(win_name)
    canvas = fig.canvas
    dpi = fig.get_dpi()
    # add a small EPS to avoid precision lost due to matplotlib's truncation
    # (https://github.com/matplotlib/matplotlib/issues/15363)
    fig.set_size_inches((width + EPS) / dpi, (height + EPS) / dpi)

    # remove white edges by set subplot margin
    plt.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax = plt.gca()
    ax.axis('off')

    polygons = []
    color = []
    for i, (bbox, label) in enumerate(zip(bboxes, labels)):
        bbox_int = bbox.astype(np.int32)
        poly = [[bbox_int[0], bbox_int[1]], [bbox_int[0], bbox_int[3]],
                [bbox_int[2], bbox_int[3]], [bbox_int[2], bbox_int[1]]]
        np_poly = np.array(poly).reshape((4, 2))
        polygons.append(Polygon(np_poly))
        color.append(bbox_color)
        label_text = class_names[
            label] if class_names is not None else f'class {label}'
        if len(bbox) > 4:
            label_text += f'|{bbox[-1]:.02f}'
        ax.text(
            bbox_int[0],
            bbox_int[1],
            f'{label_text}',
            bbox={
                'facecolor': 'black',
                'alpha': 0.8,
                'pad': 0.7,
                'edgecolor': 'none'
            },
            color=text_color,
            fontsize=font_size,
            verticalalignment='top',
            horizontalalignment='left')
        if segms is not None:
            color_mask = mask_colors[labels[i]]
            mask = segms[i].astype(bool)
            img[mask] = img[mask] * 0.5 + color_mask * 0.5

    plt.imshow(img)

    p = PatchCollection(
        polygons, facecolor='none', edgecolors=color, linewidths=thickness)
    ax.add_collection(p)

    stream, _ = canvas.print_to_buffer()
    buffer = np.frombuffer(stream, dtype='uint8')
    img_rgba = buffer.reshape(height, width, 4)
    rgb, alpha = np.split(img_rgba, [3], axis=2)
    img = rgb.astype('uint8')
    img = mmcv.rgb2bgr(img)

    if show:
        # We do not use cv2 for display because in some cases, opencv will
        # conflict with Qt, it will output a warning: Current thread
        # is not the object's thread. You can refer to
        # https://github.com/opencv/opencv-python/issues/46 for details
        if wait_time == 0:
            plt.show()
        else:
            plt.show(block=False)
            plt.pause(wait_time)
    if out_file is not None:
        mmcv.imwrite(img, out_file)

    plt.close()

    return img
