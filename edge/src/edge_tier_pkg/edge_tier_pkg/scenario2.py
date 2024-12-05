import threading
import time
import math
import numpy as np

lock = threading.Lock()

class Scenario2:
    """
    This scenario represents a little production line. The AGVs should move raw material from warehouse to workstations, 
    between workstations (semi-finished products) and from workstations to dispatch area.
    """
    def __init__(self):

        self.get_logger().info(f"Loading scenario 2...")

        self.positions = {'warehouse': [(20.5, 0.0, 0.0), (18.0, 0.0, 0.0), (15.5, 0.0, 0.0), (13.0, 0.0, 0.0)],
                              'ws1': [(6.5, 2.0, 0.0), (1.5, 2.0, 0.0)], 'ws2': [(6.5, -2.0, 0.0), (1.5, -2.0, 0.0)],
                              'ws3': [(-1.5, 2.0, 0.0), (-6.5, 2.0, 0.0), (-4.0, 3.5, 0.0)],
                              'ws4': [(-1.5, -2.0, 0.0), (-6.5, -2.0, 0.0), (-4.0, -3.5, 0.0)],
                              'fp': [(-14.5, 0.0, 0.0), (-12.0, 0.0, 0.0)]}

        self.occupancy = {'warehouse': [False, False, False, False], 'ws1': [True, True], 'ws2': [True, True],
                              'ws3': [False, True, True], 'ws4': [False, True, True], 'fp': [False, False]}

        self.count = {'agv_1': 0, 'agv_2': 0, 'agv_3': 0, 'agv_4': 0, 'agv_5': 0, 'agv_6': 0, 'agv_7': 0, 'agv_8': 0}

    def agv_warehouse(self, agv_id):

        '''
        This function supervises the positions of agvs on warehouse and moves agvs for there.
        :param agv_id: string of agv id.
        '''

        # Get the agv instance on supervisory
        for agv_instance in self.agv_instances:
            if agv_instance.agv_id == agv_id:
                break
        
        # Remain in loop until the agv reach the first position from warehouse.
        while self.run:

            # Check if the first position of warehouse is empty.
            # If true, move agv into position and break.
            i = 0
            if not self.occupancy['warehouse'][i]:
                lock.acquire()
                self.occupancy['warehouse'][i] = True
                lock.release()
                self.call_mpc(agv_instance, self.positions['warehouse'][i][0], 
                                            self.positions['warehouse'][i][1], 
                                            self.positions['warehouse'][i][2])
                break
            
            # If the first position is busy, look for the next available.
            # When find it, break and after move agv for there.
            while self.run:
                i += 1
                if not self.occupancy['warehouse'][i]:
                    break
            lock.acquire()
            self.occupancy['warehouse'][i] = True
            lock.release()
            self.call_mpc(agv_instance, self.positions['warehouse'][i][0], 
                                        self.positions['warehouse'][i][1], 
                                        self.positions['warehouse'][i][2])

            # Wait for the next position to be empty and do nothing.
            while self.occupancy['warehouse'][i - 1]:
                pass
            lock.acquire()
            self.occupancy['warehouse'][i] = False
            lock.release()

            # Go to beginning of loop.


    def agv_to_ws(self, ws, agv_id, side):

        """
        This function moves the agv from warehouse to work station.
        :param ws: workstation of destination.
        :param agv_id: string of agv id.
        :param side: side of workstation.
        """
        # Get the agv instance on supervisory and move it to workstation.
        for agv_instance in self.agv_instances:
            if agv_instance.agv_id == agv_id:
                self.call_mpc(agv_instance, self.positions[ws][side][0],
                                            self.positions[ws][side][1],
                                            self.positions[ws][side][2])
                break

    def agv_from_ws_to_ws(self, agv_id, agv_dep, ws_from, ws_to):

        """
        This function moves the agv between workstations.
        :param agv_id: string of avg id.
        :param agv_dep: string of agv id which the current is dependent.
        :param ws_from: departure workstation.
        :param ws_to: workstation of destination.
        """

        # Get the agv instance on supervisory .
        for agv_instance in self.agv_instances:
            if agv_instance.agv_id == agv_id:
                break

        while self.run:
            # When the other agv which the current is dependent counts two movements, 
            # it means that it has gone to other workstation and is back. The current agv 
            # waits until the material is processed. Then, it moves to the destination workstation, 
            # waits the material to be caught and back to the origin workstation.
            if self.count[agv_dep]:
                if not self.count[agv_dep] % 2:
                    time.sleep(5)
                    self.call_mpc(agv_instance, self.positions[ws_to][0][0], 
                                                self.positions[ws_to][0][1],
                                                self.positions[ws_to][0][2])
                    # Calculate distance until the destination to check if the agv has reched the target.
                    while self.calc_dist(agv_instance.pose, self.positions[ws_to][0]) > 0.3:
                        pass
                    self.count[agv_id] += 1
                    time.sleep(5)
                    self.call_mpc(agv_instance, self.positions[ws_from][1][0], 
                                                self.positions[ws_from][1][1],
                                                self.positions[ws_from][1][2])            
                    while self.calc_dist(agv_instance.pose, self.positions[ws_from][1]) > 0.3:
                        pass
                    self.count[agv_id] += 1

    def agv_from_ws_to_fp(self, agv_id, agv_dep_1, agv_dep_2, ws_from):

        """
        This function moves the agv from workstation to finished products.
        :param agv_id: string of avg id.
        :param agv_dep_1: string of first agv id which the current is dependent.
        :param agv_dep_2: string of second agv id which the current is dependent.
        :param ws_from: departure workstation.
        """

        # Get the agv instance on supervisory.
        for agv_instance in self.agv_instances:
            if agv_instance.agv_id == agv_id:
                break

        while self.run:
            # When the others two agvs which the current is dependent counts two movements, 
            # it means that these has gone to other workstation or warehouse and are back. The current agv 
            # waits until the material is processed. Then, it moves to the finished products, 
            # waits the material to be caught and back to the origin workstation.
            if self.count[agv_dep_1] and self.count[agv_dep_2]:
                if self.count[agv_dep_1] % 2 and self.count[agv_dep_2] % 2:
                    time.sleep(5)
                    # Remain in loop until the agv reach the first position from finished products.
                    while self.run:
                        if not self.occupancy['fp'][0]:
                            lock.acquire()
                            self.occupancy['fp'][0] = True
                            lock.release()
                            self.call_mpc(agv_instance, self.positions['fp'][0][0],
                                                        self.positions['fp'][0][1],
                                                        self.positions['fp'][0][2])
                            while self.calc_dist(agv_instance.pose, self.positions['fp'][0]) > 0.3:
                                pass
                            self.count[agv_id] += 1
                            break
                        else:
                            lock.acquire()
                            self.occupancy['fp'][1] = True
                            lock.release()
                            self.call_mpc(agv_instance, self.positions['fp'][1][0],
                                                        self.positions['fp'][1][1],
                                                        self.positions['fp'][1][2])
                            while self.calc_dist(agv_instance.pose, self.positions['fp'][1]) > 0.3:
                                if not self.occupancy['fp'][0]:
                                    break
                            while self.occupancy['fp'][0]:
                                pass
                            lock.acquire()
                            self.occupancy['fp'][1] = False
                            lock.release()

                    time.sleep(5)
                    
                    # Go back to workstation.
                    self.call_mpc(agv_instance, self.positions[ws_from][1][0],
                                                self.positions[ws_from][1][1],
                                                self.positions[ws_from][1][2])
                    while self.calc_dist(agv_instance.pose, self.positions['fp'][0]) <= 1.0:
                        pass
                    lock.acquire()
                    self.occupancy['fp'][0] = False
                    lock.release()
                    while self.calc_dist(agv_instance.pose, self.positions[ws_from][1]) > 0.3:
                        pass
                    self.count[agv_id] += 1


    def agv_loop(self, agv_id, ws, side):

        """
        This function puts the agv in a loop between the warehouse and workstation.
        :param ws: origin workstation.
        :param side: side of workstation.
        """

        # Get the agv instance on supervisory.
        for agv_instance in self.agv_instances:
                if agv_instance.agv_id == agv_id:
                    break
        
        # Start the loop calling cyclically the functions agv_warehouse and agv_to_ws.
        while self.run:
            self.occupancy[ws][side] = False
            self.agv_warehouse(agv_id)
            while self.calc_dist(agv_instance.pose, self.positions['warehouse'][0]) > 0.3:
                pass
            self.count[agv_id] += 1
            time.sleep(5)
            self.agv_to_ws(ws, agv_id, side)
            while self.calc_dist(agv_instance.pose, self.positions['warehouse'][0]) <= 1.0:
                pass
            lock.acquire()
            self.occupancy['warehouse'][0] = False
            lock.release()
            while self.calc_dist(agv_instance.pose, self.positions[ws][side]) > 0.3:
                pass
            self.count[agv_id] += 1
            self.occupancy[ws][side] = True
            time.sleep(5)

    def call_mpc(self, agv_instance, x, y, theta):
        agv_instance.call_mpc('move', float(x), float(y), float(theta))

    def calc_dist(self, pose, ref):
        x, y, theta = ref[0], ref[1], math.radians(ref[2])
        return np.linalg.norm(np.array(pose) - np.array([x,y,theta]))

    def start_scenario(self):
        t = []
        self.run = True
        t.append(threading.Thread(target=self.agv_loop, args=('agv_1', 'ws1', 0)))
        t.append(threading.Thread(target=self.agv_loop, args=('agv_2', 'ws2', 0)))
        t.append(threading.Thread(target=self.agv_loop, args=('agv_5', 'ws3', 2)))
        t.append(threading.Thread(target=self.agv_loop, args=('agv_6', 'ws4', 2)))
        t.append(threading.Thread(target=self.agv_from_ws_to_ws, args=('agv_3', 'agv_1', 'ws1', 'ws3')))
        t.append(threading.Thread(target=self.agv_from_ws_to_ws, args=('agv_4', 'agv_2', 'ws2', 'ws4')))
        t.append(threading.Thread(target=self.agv_from_ws_to_fp, args=('agv_7', 'agv_3', 'agv_5', 'ws3')))
        t.append(threading.Thread(target=self.agv_from_ws_to_fp, args=('agv_8', 'agv_4', 'agv_6', 'ws4')))

        for thread in t:
            thread.start()

    def stop_scenario(self):
        self.run = False
        for agv_instance in self.agv_instances:
            agv_instance.call_mpc('stop', 0.0, 0.0, 0.0)