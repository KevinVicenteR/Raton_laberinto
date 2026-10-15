"""Experimental observed-map sweep. No access to hidden maze segments or true pose.

The only map prior accepted here is Case.known, explicitly supplied in mode 4.
Unknown-map walls come from recorded cell observations; pose is integrated from
encoder distance and gyro increments. Physics still checks contact independently.
"""
import math
import sim_extremos as S

class ObservedMapSim(S.Sim):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.nav_x=(self.c.start[0]+.5)*self.pitch
        self.nav_y=(self.c.start[1]+.5)*self.pitch-(self.c.robot.length_front-self.c.robot.length_back)/2
        self.nav_yaw0=self.th
        self.nav_walls={}
        self.nav_seen=0
        if self.c.known is not None:
            maze=self.c.known
            for y in range(self.rows):
                for x in range(self.cols):
                    r=self.rows-1-y
                    for d,value in enumerate((maze[2*r][4*x+2]=='-',maze[2*r+1][4*x+4]=='|',maze[2*r+2][4*x+2]=='-',maze[2*r+1][4*x]=='|')):
                        if value:self.nav_walls[x,y,d]=True

    def try_move(self,x,y,heading):
        before=self.travelled;old=self.th
        moved=super().try_move(x,y,heading)
        # Encoder and gyro measurements only; do not read x/y or collision geometry.
        distance=self.travelled-before
        yaw=(old+heading)/2-self.nav_yaw0+math.pi/2
        self.nav_x+=distance*math.cos(yaw);self.nav_y+=distance*math.sin(yaw)
        return moved

    def observed_segments(self):
        for sample in self.sensed[self.nav_seen:]:
            _,pos,heading,left,front,right,*_=sample
            for d,value in (((heading+3)%4,left),(heading,front),((heading+1)%4,right)):
                if value:self.nav_walls[pos[0],pos[1],d]=True
        self.nav_seen=len(self.sensed)
        segments=[];half=self.c.wall/2;p=self.pitch
        for (x,y,d) in self.nav_walls:
            if abs((x+.5)*p-self.nav_x)>350 or abs((y+.5)*p-self.nav_y)>350:continue
            # Inflate a cell boundary to its configured physical wall thickness.
            if d in (0,2):
                yy=(y+(d==0))*p
                lo,hi=(x*p-half,yy-half),((x+1)*p+half,yy+half)
            else:
                xx=(x+(d==1))*p
                lo,hi=(xx-half,y*p-half),(xx+half,(y+1)*p+half)
            a,b=lo,hi;c=(lo[0],hi[1]);e=(hi[0],lo[1])
            segments.extend(((a,c),(c,b),(b,e),(e,a)))
        return segments

    def clear_observed_pose(self,segments,x,y,angle):
        body=self.c.robot;c,s=math.cos(angle),math.sin(angle);margin=3
        points=[(x+f*c-l*s,y+f*s+l*c) for f,l in ((body.length_front+margin,body.width/2+margin),(body.length_front+margin,-body.width/2-margin),(-body.length_back-margin,-body.width/2-margin),(-body.length_back-margin,body.width/2+margin))]
        for a,b in segments:
            if any(S.segs_cross(points[i],points[(i+1)%4],a,b) for i in range(4)):return False
            dx,dy=a[0]-x,a[1]-y
            if -body.length_back-margin<=dx*c+dy*s<=body.length_front+margin and abs(-dx*s+dy*c)<=body.width/2+margin:return False
        return True

    def mapped_turn_clear(self,segments,degrees,entry):
        angle=self.th-self.nav_yaw0+math.pi/2
        sign=1 if degrees>0 else -1
        split=math.radians(sign*self.c.fw.split_deg)
        x,y=self.nav_x,self.nav_y
        def clear(dx,dy,da):return self.clear_observed_pose(segments,x+dx,y+dy,angle+da)
        # 1 mm / 1 degree sampling plus a 3 mm envelope.
        count=max(1,math.ceil(abs(entry)))
        for i in range(count+1):
            at=entry*i/count
            if not clear(at*math.cos(angle),at*math.sin(angle),0):return False
        ex,ey=entry*math.cos(angle),entry*math.sin(angle)
        count=math.ceil(abs(math.degrees(split)))
        for i in range(count+1):
            if not clear(ex,ey,split*i/count):return False
        advance=self.c.fw.split_advance;count=math.ceil(advance)
        for i in range(count+1):
            d=advance*i/count
            if not clear(ex+d*math.cos(angle+split),ey+d*math.sin(angle+split),split):return False
        dx,dy=ex+advance*math.cos(angle+split),ey+advance*math.sin(angle+split)
        count=math.ceil(90-self.c.fw.split_deg)
        for i in range(count+1):
            da=split+math.radians(sign*(90-self.c.fw.split_deg))*i/count
            if not clear(dx,dy,da):return False
        return True

    def prepare_turn(self,degrees):
        if not self.action_allowed(degrees):raise S.MotionFailure('giro rechazado por riesgo')
        segments=self.observed_segments()
        # Prefer nominal entry; bounds are explicit controller moves, never a pose reset.
        entry=next((e for e in (0,-5,-10,-15,5,10,15,-20,20,-25,25,-30,30) if self.mapped_turn_clear(segments,degrees,e)),None)
        if entry is None:raise S.MotionFailure('barrido bloqueado por paredes observadas')
        before=self.travelled
        if entry<0:yield from self.back(-entry)
        elif entry>0:yield from self.drive(entry,0)
        if abs(self.travelled-before-entry)>3:raise S.MotionFailure('ajuste de entrada incompleto')
