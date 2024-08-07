"""
This script can help you to deploy a complete policy for your device.
"""
import tensorflow as tf
import os
import numpy as np
from copy import deepcopy

# import os
# os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

def mkdir(path: str):
    """
    create a directory in current path.

    Args:
        path (_type_:str): name of directory.

    Returns:
        _type_: str or None: path of directory.
    """
    current_path = os.getcwd()
    # print(current_path)
    path = os.path.join(current_path, path)
    if not os.path.exists(path):
        os.makedirs(path)
        return path
    else:
        None


class RunningMeanStd:
    # Dynamically calculate mean and std
    def __init__(self, shape, name):  # shape:the dimension of input data
        self.n = 0
        self.mean = np.zeros(shape)
        self.S = np.zeros(shape)
        self.std = np.sqrt(self.S)

        self.shape = shape

        self.name = name

    def update(self, x):
        x = np.asarray(x)
        size = x.shape[0]

        for i in range(size):
            self.update_(x[i])

    def update_(self, x):
        self.n += 1
        if self.n == 1:
            self.mean = x
            self.std = x
        else:
            old_mean = deepcopy(self.mean)
            self.mean = old_mean + (x - old_mean) / self.n
            self.S = self.S + (x - old_mean) * (x - self.mean)
            self.std = np.sqrt(self.S / self.n)

    def save(self, path):
        # mkdir(path)
        all_path_name = os.path.join(path, self.name)
        mkdir(all_path_name)
        np.save(os.path.join(all_path_name, 'mean.npy'), self.mean)
        np.save(os.path.join(all_path_name, 'std.npy'), self.std)
        np.save(os.path.join(all_path_name, 'n.npy'), self.n)

    def load(self, path):
        all_path_name = os.path.join(path, self.name)
        self.mean = np.load(os.path.join(all_path_name, 'mean.npy'))
        self.std = np.load(os.path.join(all_path_name, 'std.npy'))
        self.n = np.load(os.path.join(all_path_name, 'n.npy'))


class Normalization:
    def __init__(self, obs_shape_dict: dict):
        self.running_ms = {}
        for key, value in obs_shape_dict.items():
            self.running_ms[key] = RunningMeanStd(value, name=key)

    def __call__(self, x: dict, update=True):
        # Whether to update the mean and std,during the evaluating,update=Flase
        new_x = {}
        if update:
            for key, value in x.items():
                self.running_ms[key].update(value)

        for key, value in x.items():
            new_x[key] = (value - self.running_ms[key].mean) / (self.running_ms[key].std + 1e-8)

        return new_x

    def save(self, path):
        for key, value in self.running_ms.items():
            value.save(path)

    def load(self, path):
        for key, value in self.running_ms.items():
            value.load(path)


class CompletePolicy:
    # Complete policy for your device.
    # TODO: store the input shape of the network
    def __init__(self,
                 actor,
                 obs_shape_dict,
                 checkpoint_path,
                 using_obs_scale=False,
                 ):

        """
        Complete policy for your device.

        args:
            actor (tf.keras.Model): actor model.
            checkpoint_path (str): checkpoint path.
            using_obs_scale (bool): whether using observation scale.
        """

        self.obs_dict = obs_shape_dict

        self.actor = actor()

        self.initialize_actor()

        actor_model_path = os.path.join(checkpoint_path, 'actor.h5')

        self.actor.load_weights(actor_model_path)

        self.normalizer = Normalization(obs_shape_dict)

        self.using_obs_scale = using_obs_scale

        if using_obs_scale:
            norm_path = os.path.join(checkpoint_path, 'scaler')
            self.normalizer.load(norm_path)

        self._network_process_info = {
            'actor': {},
            'critic': {},
        }  # 网络输入数据处理信息

    def initialize_actor(self):
        """
        用于初始化actor，比如说在rnn系统模型里面，某些输入需要额外处理维度。

        #TODO：在2.1版本中逐步将网络输入配置融入到网络定义中。
        """

        # 判断网络类型
        actor_rnn_flag = getattr(self.actor, 'rnn_flag', False)

        # RNN输入时候维度的处理

        self.actor_expand_dims_idx = []

        if actor_rnn_flag:
            self._network_process_info['actor']['rnn_flag'] = True
            idx = 0
            actor_input_names = self.actor.input_name

            for name in actor_input_names:
                if 'hidden' in name:
                    pass
                else:
                    self.actor_expand_dims_idx.append(idx)
                idx += 1
            self.actor_expand_dims_idx = tuple(self.actor_expand_dims_idx)

        self.initialize_network(
            model=self.actor,
            expand_dims_idx=self.actor_expand_dims_idx,
        )

    def initialize_network(self, model, expand_dims_idx=None):
        """

        初始化网络参数。

        Args:
            model (_type_): _description_
            expand_dims_idx (_type_, optional): _description_. Defaults to None.
        """

        input_data_name = model.input_name

        # create tensor according to input data name
        input_data = []

        for name in input_data_name:
            shape = self.obs_dict[name]

            data = tf.zeros(shape=shape, dtype=tf.float32)
            input_data.append(data)
        if expand_dims_idx is not None:
            for idx in expand_dims_idx:
                input_data[idx] = tf.expand_dims(input_data[idx], axis=1)

        model(*input_data)

    def get_action(self, obs):

        if self.using_obs_scale:
            obs_ = self.normalizer(deepcopy(obs), update=False)
            obs = deepcopy(obs_)

        input_data = []

        # 获取输入数据
        for name in self.actor.input_name:
            data = tf.cast(obs[name], dtype=tf.float32)
            input_data.append(data)

        for idx in self.actor_expand_dims_idx:
            input_data[idx] = tf.expand_dims(input_data[idx], axis=1)

        actor_out = self.actor(*input_data)

        action = actor_out[0].numpy()

        return {'action': action}


# class Actor_net(tf.keras.Model):

#     def __init__(self):
#         super(Actor_net, self).__init__()

#         # self.dense1 = tf.keras.layers.Dense(16, activation='relu',kernel_initializer=tf.keras.initializers.orthogonal())
#         # self.dense2 = tf.keras.layers.Dense(32, activation='relu',kernel_initializer=tf.keras.initializers.orthogonal())

#         self.action_layer1 = tf.keras.layers.Dense(64, activation='tanh',
#                                                    kernel_initializer=tf.keras.initializers.orthogonal())
#         self.action_layer2 = tf.keras.layers.Dense(64, activation='tanh',
#                                                    kernel_initializer=tf.keras.initializers.orthogonal())
#         self.action_layer = tf.keras.layers.Dense(4)

#         self.value_layer1 = tf.keras.layers.Dense(64, activation='relu',
#                                                   kernel_initializer=tf.keras.initializers.orthogonal())
#         self.value_layer2 = tf.keras.layers.Dense(64, activation='relu',
#                                                   kernel_initializer=tf.keras.initializers.orthogonal())
#         self.value_layer = tf.keras.layers.Dense(1)

#         self.output_info = {'action': (4,), 'value': (1,)}
#         # self.output_info = {'action': (4,),}

#         self.input_name = ('obs',)

#         self.optimizer_info = {
#             'type': 'Adam',
#             'args': {'learning_rate': 3e-4,
#                      'epsilon': 1e-5,
#                      'clipnorm': 0.5,
#                      }
#         }

#     @tf.function
#     def call(self, obs):
#         # x = self.dense1(obs)
#         # x = self.dense2(x)
#         action1 = self.action_layer1(obs)
#         action2 = self.action_layer2(action1)
#         action = self.action_layer(action2)
#         # logstd1 = self.log_std1(x)
#         # logstd = self.log_std(logstd1)

#         value1 = self.value_layer1(obs)
#         value2 = self.value_layer2(value1)
#         value = self.value_layer(value2)
#         # log_std = self.log_std(x)

#         return (action, value,)
#         # return (action,)

#     def reset(self):
#         pass


class Actor_net(tf.keras.Model):

    def __init__(self):
        super(Actor_net, self).__init__()

        self.dense1 = tf.keras.layers.Dense(64, activation='relu',kernel_initializer=tf.keras.initializers.orthogonal())
        self.dense2 = tf.keras.layers.Dense(64, activation='relu',kernel_initializer=tf.keras.initializers.orthogonal())
        self.action_layer = tf.keras.layers.Dense(4,kernel_initializer=tf.keras.initializers.orthogonal())
        

        self.learning_rate = 0.001#env_Args["actor_lr"]

        self.output_info = {'action': (4,),}
        # self.output_info = {'action': (4,),}
        

        self.input_name = ('obs',)

        self.optimizer_info = {
            'type': 'Adam',
            'args': {'learning_rate': 3e-4,
                     'epsilon': 1e-5,
                        'clipnorm': 0.5,
                     }
        }
        #self.weight_path = actor_file #'actor.h5'
        
        # input_ = tf.keras.layers.Input((18,))
        # x = self.dense1(input_)
        # x = self.dense2(x)
        # x = self.dense3(x)
        # out = self.action_layer(x)
        
        # model = tf.keras.Model(
        #     inputs=input_,
        #     outputs=out
        # )
        
        # model.load_weights('/home/zmn/ytppo7/quad_train/debug3/cache/PPO/actor.h5')

    @tf.function
    def call(self, obs, mask=None):
        x = self.dense1(obs)
        x = self.dense2(x)
        action = self.action_layer(x)
        # logstd1 = self.log_std1(x)
        # logstd = self.log_std(logstd1)
        
        # value_1 = self.value_layer1(x)
        # value = self.value_layer(value_1)
        #log_std = self.log_std(x)

        return (action, )
        # return (action,)

    def reset(self):
        pass


obs_shape_dict = {
    'obs': (1, 18)
}

policy = CompletePolicy(
    actor=Actor_net,
    obs_shape_dict=obs_shape_dict,
    checkpoint_path='/home/jetson/airdocking/src/velocity_control/scripts/model_obstacle/4/6901',
    #checkpoint_path='/home/jetson/airdocking/src/velocity_control/scripts/model_obstacle/N2Smooth6/2561',
    using_obs_scale=False,
)


def modelPredict(state_error):
    state = {
        'obs': state_error.reshape(1,18)
    }

    out = policy.get_action(state)
    action = out['action']

    action = np.squeeze(action)

    return action