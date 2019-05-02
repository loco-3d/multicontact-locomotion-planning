import os
from ast import literal_eval

class Status:
    def __init__(self,filename=None):
        self.q_init = []
        self.q_goal = []
        self.planning_success = False
        self.cg_success= False
        self.cg_reach_goal= False
        self.cg_too_many_states= None
        self.gen_cs_success= False
        self.centroidal_success= False
        self.wholebody_success= False
        self.wholebody_reach_goal= False
        self.motion_valid= False    
        if filename is not None:
            self.__loadFromFile__(filename)
            
        
    def __loadFromFile__(self,filename):
        f = open(filename,"r")
        for line in f.readlines():
            tab = line.rstrip("\n").split(" ")
            if tab[0].startswith("q_init"):
                self.q_init = literal_eval(line.lstrip('q_init= ').rstrip(' '))
            if tab[0].startswith("q_goal"):
                self.q_goal = literal_eval(line.lstrip('q_goal= ').rstrip(' '))
            if tab[0].startswith("Planning_success"):
                self.planning_success = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("cg_success"):
                self.cg_success = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("cg_reach_goal"):
                self.cg_reach_goal = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("cg_too_many_states"):
                self.cg_too_many_states = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("gen_cs_success"):
                self.gen_cs_success = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("centroidal_success"):
                self.centroidal_success = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("wholebody_success"):
                self.wholebody_success = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("wholebody_reach_goal"):
                self.wholebody_reach_goal = literal_eval(tab[1].rstrip(' '))
            if tab[0].startswith("motion_valid"):
                self.motion_valid = literal_eval(tab[1].rstrip(' '))
    
    
                    