from hpp_spline import bezier, bezier6, polynom, exact_cubic, curve_constraints, spline_deriv_constraint, from_bezier
import inspect

class PolyBezier:
    
    def __init__(self,curves):
        if not isinstance(curves,list): # deal with single bezier curve input
            curves = [curves]
        self.curves=curves
        self.times=[]
        self.times +=[0]
        self.d_curves=[]
        self.dd_curves=[]
        self.jerk_curves=[]
        for i in range(len(curves)):
            if not isinstance(curves[i],bezier):
                raise TypeError("PolyBezier must be called with a list of bezier curves (or a single bezier curve)")
            self.times+=[curves[i].max() + self.times[-1]]
            
    def findInterval(self,t):
        if t>self.times[-1] or t<0:
            raise ValueError("Parameter is outside of definition range of the curves, t = "+str(t) )       
        for cit in range(len(self.times)):
            if t<=self.times[cit+1]:
                return cit  
        raise ValueError("Error in times intervals for t = "+str(t))        
            
    def findIntervalAdjustTime(self,t):
        id = self.findInterval(t)
        t -= self.times[id]
        return id,t
    
    def getBezierAt(self,t):
        id = self.findInterval(t)
        return self.curves[id]
       
    def __call__(self,t):
        id = self.findInterval(t)
        tc = t-self.times[id]
        return self.curves[id](tc)
            
    def numCurves(self):
        return len(self.curves)
    
    def length(self):
        return self.times[-1]

    def max(self):
        return self.length()
    
    def lengthNonZero(self):
        length = 0
        for c in self.curves:
            if c.degree > 0:
                length += (c.max() - c.min())
        return length
    
    def isInFirst(self,t):
        id = self.findInterval(t)
        return id==0
    
    def isInLast(self,t):
        id = self.findInterval(t)
        return id == (len(self.curves)-1)

    def idFirstNonZero(self):
        id_nonZero = 0
        while (self.curves[id_nonZero].degree == 0):
            id_nonZero += 1  
        return id_nonZero
    
    def idLastNonZero(self):
        id_nonZero = len(self.curves)-1
        while (self.curves[id_nonZero].degree == 0):
            id_nonZero -= 1
        return id_nonZero
    
    def isInFirstNonZero(self,t):
        id = self.findInterval(t)
        id_nonZero = self.idFirstNonZero()
        return id <= id_nonZero
    
    def isInLastNonZero(self,t):
        id = self.findInterval(t)
        id_nonZero = self.idLastNonZero()
        return id >= id_nonZero  
    
    def firstNonZero(self):
        return self.curves[self.idFirstNonZero()]
    
    def lastNonZero(self):
        return self.curves[self.idLastNonZero()]
        
    
    def isInExtermities(self,t):
        return self.isInFirst(t) or self.isInLast(t)
    
    def computeDerivates(self):
        if len(self.d_curves)!=len(self.curves) or len(self.dd_curves)!=len(self.curves) :
            self.d_curves=[]
            self.dd_curves=[]
            for c in self.curves:   
                self.d_curves += [c.compute_derivate(1)]
                self.dd_curves += [c.compute_derivate(2)]  
                self.jerk_curves +=[c.compute_derivate(3)]
        else:
            print "Derivatives curves were already computed"
    
    def d(self,t):
        id,t = self.findIntervalAdjustTime(t)  
        if len(self.d_curves) == len(self.curves):
            return self.d_curves[id](t) 
        else : 
            return self.curves[id].derivate(t,1)
        
    
    def dd(self,t):
        id,t = self.findIntervalAdjustTime(t)         
        if len(self.dd_curves) == len(self.curves):
            return self.dd_curves[id](t) 
        else : 
            return self.curves[id].derivate(t,2)   
        
    def jerk(self,t):
        id,t = self.findIntervalAdjustTime(t)         
        if len(self.jerk_curves) == len(self.curves):
            return self.jerk_curves[id](t) 
        else : 
            return self.curves[id].derivate(t,3)                