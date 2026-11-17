#!/usr/bin/env python3
"""Fresh per-contact wrench inputs; never collapse opposing grasp forces."""
import numpy as np
from shape_mapping import vector


class ContactInputs:
    def __init__(self,specs,robot):
        if not isinstance(specs,list) or not specs:raise ValueError('at least one contact specification required')
        self.specs=[]
        self.robot=robot
        self.latest={}
        for spec in specs:
            link=spec.get('link','link1')
            if not isinstance(link,str) or not link:raise ValueError('contact link required')
            self.specs.append(dict(link=link,point=vector(spec.get('point',[0,0,0]),3).tolist(),
                                   topic=spec.get('topic','/'+robot+'/contact/wrench')))
        if len({s['topic'] for s in self.specs})!=len(self.specs):
            raise ValueError('each contact requires an individual wrench topic')

    def update(self,index,msg,received):
        self.latest[index]=(msg,received)

    def values(self,now,wall):
        contacts=[]
        for i,spec in enumerate(self.specs):
            if i not in self.latest:raise ValueError('contact wrench unavailable')
            msg,received=self.latest[i]
            stamp=msg.header.stamp.to_sec()
            if not stamp>0 or not 0<=now-stamp<=.25 or not 0<=wall-received<=.25:
                raise ValueError('contact wrench stale or future')
            frame=msg.header.frame_id.lstrip('/')
            local=frame in (spec['link'],self.robot+'/'+spec['link'])
            if not local and frame not in ('fc',self.robot+'/fc'):raise ValueError('contact frame mismatch')
            f,m=msg.wrench.force,msg.wrench.torque
            wrench=vector([f.x,f.y,f.z,m.x,m.y,m.z],6).tolist()
            contacts.append((spec['link'],spec['point'],wrench,local))
        return contacts
