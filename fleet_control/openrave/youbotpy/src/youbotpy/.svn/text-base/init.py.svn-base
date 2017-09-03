#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np

import openravepy as orpy
import tfplugin

def init(sim=False,viewer=False,env_xml=None,youbot_names=[]):
    youbotpydir = os.popen('rospack find youbotpy').read()
    print youbotpydir
    
    #RaveSetDebugLevel(0) # suppresses printing of non fatal errors
    
    env = orpy.Environment()
    if viewer:
        env.SetViewer('qtcoin')
    if env_xml:
        env.Load(env_xml)
    
    youbots = {}
    for youbot_name in youbot_names:
        youbot = env.ReadRobotURI(youbotpydir[:-1] + '/../models/robots/kuka-youbot.robot.xml')
        youbot.SetName(youbot_name)
        env.Add(youbot,True)
        youbots[youbot_name] = youbot
    
    tf_plugin=None
    if not sim:
        tf_plugin = tfplugin.TfPlugin(env,'map')
        for youbot_name in youbot_names:
            probotcontroller = orpy.RaveCreateController(env,'youbotcontroller')
            youbots[youbot_name].SetController(probotcontroller)
            tf_plugin.RegisterBody(or_body=youbots[youbot_name],tf_id=youbot_name)

    return env,youbots,tf_plugin
    


