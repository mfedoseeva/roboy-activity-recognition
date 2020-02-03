from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import tensorflow as tf

import os
GPU = True
if not GPU:
    os.environ['CUDA_VISIBLE_DEVICES'] = ''

from. import i3d

data_path = os.path.join(os.path.dirname(__file__), 'data')

_IMAGE_SIZE = 224

_SAMPLE_VIDEO_FRAMES = 79
_SAMPLE_PATHS = {
    'rgb': '',
    'flow': '',
}

_CHECKPOINT_PATHS = {
    'rgb_imagenet': os.path.join(data_path,
                                 'checkpoints/rgb_imagenet/model.ckpt'),
    'flow_imagenet': os.path.join(data_path,
                                  'checkpoints/flow_imagenet/model.ckpt'),
}

_LABEL_MAP_PATH = os.path.join(data_path, 'label_map.txt')

_NUM_CLASSES = 400

FLAGS = tf.flags.FLAGS

# change here the type of data used for prediction
tf.flags.DEFINE_string('eval_type', 'joint', 'rgb, flow, or joint')


class Predictor:
    def __init__(self):
        tf.logging.set_verbosity(tf.logging.INFO)
        self.eval_type = FLAGS.eval_type

        if self.eval_type not in ['rgb', 'flow', 'joint']:
            raise ValueError(
                'Bad `eval_type`, must be one of rgb, rgb600, flow, joint')

        self.kinetics_classes = [x.strip() for x in open(_LABEL_MAP_PATH)]

        if self.eval_type in ['rgb', 'joint']:
            # RGB input has 3 channels.
            self.rgb_input = tf.placeholder(
                tf.float32,
                shape=(1, _SAMPLE_VIDEO_FRAMES, _IMAGE_SIZE, _IMAGE_SIZE, 3))

            with tf.variable_scope('RGB'):
                self.rgb_model = i3d.InceptionI3d(
                    _NUM_CLASSES,
                    spatial_squeeze=True,
                    final_endpoint='Logits')
                rgb_logits, _ = self.rgb_model(
                    self.rgb_input,
                    is_training=False,
                    dropout_keep_prob=1.0)

            rgb_variable_map = {}
            for variable in tf.global_variables():
                if variable.name.split('/')[0] == 'RGB':
                    rgb_variable_map[variable.name.replace(
                        ':0', '')] = variable

            self.rgb_saver = tf.train.Saver(
                var_list=rgb_variable_map, reshape=True)

        if self.eval_type in ['flow', 'joint']:
            # Flow input has only 2 channels.
            self.flow_input = tf.placeholder(
                tf.float32, shape=(1,
                                   _SAMPLE_VIDEO_FRAMES,
                                   _IMAGE_SIZE,
                                   _IMAGE_SIZE,
                                   2))
            with tf.variable_scope('Flow'):
                self.flow_model = i3d.InceptionI3d(
                    _NUM_CLASSES,
                    spatial_squeeze=True,
                    final_endpoint='Logits')
                flow_logits, _ = self.flow_model(
                    self.flow_input, is_training=False, dropout_keep_prob=1.0)
            flow_variable_map = {}
            for variable in tf.global_variables():
                if variable.name.split('/')[0] == 'Flow':
                    flow_variable_map[variable.name.replace(
                        ':0', '')] = variable
            self.flow_saver = tf.train.Saver(
                var_list=flow_variable_map, reshape=True)

        if self.eval_type == 'rgb':
            self.model_logits = rgb_logits
        elif self.eval_type == 'flow':
            self.model_logits = flow_logits
        else:
            self.model_logits = rgb_logits + flow_logits
        self.model_predictions = tf.nn.softmax(self.model_logits, axis=1)

        self.session = tf.Session()
        os.environ['CUDA_VISIBLE_DEVICES'] = '0'
        self.restore_model()

    def restore_model(self):
        if self.eval_type in ['rgb', 'joint']:
            self.rgb_saver.restore(
                self.session, _CHECKPOINT_PATHS['rgb_imagenet'])
            tf.logging.info('RGB checkpoint restored')

        if self.eval_type in ['flow', 'joint']:
            self.flow_saver.restore(
                self.session, _CHECKPOINT_PATHS['flow_imagenet'])
            tf.logging.info('Flow checkpoint restored')

    def predict(self, rgb_sample, flow_sample):
        feed_dict = {}
        feed_dict[self.rgb_input] = rgb_sample
        feed_dict[self.flow_input] = flow_sample

        out_logits, out_predictions = self.session.run(
            [self.model_logits, self.model_predictions], feed_dict=feed_dict)

        out_logits = out_logits[0]
        out_predictions = out_predictions[0]
        sorted_indices = np.argsort(out_predictions)[::-1]

        print('Norm of logits: %f' % np.linalg.norm(out_logits))
        print('\nTop 5 classes and probabilities JOINT')
        for index in sorted_indices[:5]:
            print(f"{out_predictions[index]} {out_logits[index]} {self.kinetics_classes[index]}")

        # return top 3 predictions
        result_predictions = []
        result_labels = []
        for ind in sorted_indices[:3]:
            result_predictions.append(out_predictions[ind])
            result_labels.append(self.kinetics_classes[ind])
        return result_predictions, result_labels

    def close(self):
        self.session.close()


if __name__ == '__main__':

    # basic test
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--rgb_sample_path',
                        default=os.path.join(
                            data_path,
                            'v_CricketShot_g04_c01_rgb.npy'),
                        help='directory and filename of rgb input')
    parser.add_argument('--flow_sample_path',
                        default=os.path.join(
                            data_path,
                            'v_CricketShot_g04_c01_flow.npy'),
                        help='directory and filename of flow input')
    _SAMPLE_PATHS = {
        'rgb': '',
        'flow': '',
    }

    args = parser.parse_args()
    _SAMPLE_PATHS['rgb'] = args.rgb_sample_path
    _SAMPLE_PATHS['flow'] = args.flow_sample_path

    rgb_sample = np.load(_SAMPLE_PATHS['rgb'])
    tf.logging.info('RGB data loaded, shape=%s', str(rgb_sample.shape))
    flow_sample = np.load(_SAMPLE_PATHS['flow'])
    tf.logging.info('Flow data loaded, shape=%s', str(flow_sample.shape))

    predictor = Predictor()
    predictor.predict(rgb_sample, flow_sample)
