#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy

import openravepy as orpy
import IPython

class TfPlugin:

    def __init__(self, env, world_tf_id):
        self.env = env
        self.world_tf_id = world_tf_id

        self.tfplugin = orpy.RaveCreateSensor(self.env,'tfplugin tfplugin '+self.world_tf_id)
        env.Add(self.tfplugin)

    def RegisterBody(self, or_body, tf_id):
        self.tfplugin.SendCommand('RegisterBody ' + or_body.GetName() + ' ' + tf_id)

    def UnregisterBody(self, or_body):
        self.tfplugin.SendCommand('UnregisterBody ' + or_body.GetName())

    def Pause(self):
        self.tfplugin.SendCommand('Pause')

    def Resume(self):
        self.tfplugin.SendCommand('Resume')

    def Clear(self):
        self.tfplugin.SendCommand('Clear')
