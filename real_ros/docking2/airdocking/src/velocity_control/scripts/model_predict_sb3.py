from stable_baselines3 import PPO

#model_file = "/home/fyt/project/quad_raisim/test/test_sb3/168644000.zip" 119
model_file = "/home/fyt/project/quad_raisim/test/test_sb3/211652000.zip" #119
#model_file = "/home/fyt/project/quad_raisim/test/test_sb3/211680000.zip" 80
#211680000
""
model = PPO.load(model_file)

def modelPredict(state_error):
    action, _ = model.predict(state_error)
    return action
