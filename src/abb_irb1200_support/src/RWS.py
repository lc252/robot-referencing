# A Robot Web Services (RWS) class for ABB robots from https://github.com/andreasstrom/RWS-wrapper


from requests.auth import HTTPDigestAuth
from requests import Session, RequestException
from ast import literal_eval
import json, time, pathlib

class RWS:
    """Class for communicating with RobotWare through Robot Web Services (ABB's Rest API).
    This class was built on the foundation of the work done in https://github.com/prinsWindy/ABB-Robot-Machine-Vision, and have been modified and expanded on as a tool in my master's thesis work.
    "For the RW 7.0 release, RWS has introduced several breaking changes to include the following requirements" -> Updates
    """

    def __init__(self, base_url, username='Default User', password='robotics', headers = {'Content-Type':'application/x-www-form-urlencoded;v=2.0'}, headers2 = {'accept': 'application/hal+json;v=2.0'}, verify = False, task = 'T_ROB1', module = 'arrs_mod'):
        self.base_url = base_url
        self.username = username
        self.password = password
        self.session = Session() # create persistent HTTP communication
        self.session.auth = HTTPDigestAuth(self.username, self.password)
        self.headers = headers
        self.headers_json = headers2
        self.verify = verify
        self.task = task
        self.module = module

    def is_connected(self):
        try:
            self.session.get(self.base_url + '/rw/rapid/execution?json=1', timeout=0.2)
        except RequestException as e:
            print(e)
            return False
        print("Connected to Robot")
        return True
    
    def get_joints(self):
        if self.is_connected():
            resp = self.session.get(self.base_url + '/rw/motionsystem/mechunits/ROB_1/jointtarget?json=1')
            _dict = json.loads(resp.text)
            data = _dict["_embedded"]["_state"][0]
            joints = [float(data[f"rax_{i}"]) for i in range(1,7)]
            return joints
        else:
            return [0, 0, 0, 0, 0, 0]

    def get_modules(self, ignore=True, api=False):
        """"
        Get name of non-SysMod loaded modules
        """
        resp = self.session.get(f'{self.base_url}/rw/rapid/tasks/{self.task}/modules', headers=self.headers_json, auth=self.session.auth, verify=self.verify)
        if resp.status_code == 200:
            v = [mod["name"] for mod in json.loads(resp.text)["state"] if ignore and mod["type"] != "SysMod"]
            return (v, resp) if api else v
        else:
            print(f"Call might have failed\nstatus: {resp.status_code}\nbody: {resp.text}")
            return resp

    def load_RAPID(self, path='$HOME/'):
        """
        loads RAPID _program_ from path
        """
        resp = self.session.post(f'{self.base_url}/rw/rapid/tasks/{self.task}/program/load', headers=self.headers, data={'progpath': path}, auth=self.session.auth, verify=self.verify)
        return resp

    def write_array(self, name, li, type='VAR', subtype='jointtarget', mod='arrs_mod', path=''):
        """
        Method for adding array into module of saved arrays
        NB! li (the values) must be formatted correctly (according to RAPID)
        NB! will overwrite variables
        NB! no variable name can be the postscript of an earlier variable, such a variable will be overwritten
        NB! SLOW code
        """
        p=pathlib.Path(path,f"{mod}.modx")
        try: 
            with open(p, 'r+') as f:
                lines = f.readlines()
                f.seek(0)
                for line in lines: #did not manage to do this in the while below
                    if f"{name}{{" not in line:
                        f.write(line)
                f.truncate()
                f.seek(0)
                pos, text = 0, ''
                while True: #SO magic
                    # save last line value and cursor position
                    prev_pos, pos = pos, f.tell()
                    prev_text, text = text, f.readline()
                    if text == '':
                        break
                f.seek(prev_pos, 0) # replace cursor to the last line
                if subtype in ['speeddata', 'zonedata']:
                    f.write(f"\t{type} {subtype} {name}{{{len(li)}}} := {str(li)};\n".replace("'",""))
                else:
                    f.write(f"\t{type} {subtype} {name}{{{len(li)}}} := {str(li)};\n")
                f.write(prev_text) # write old last line
        except OSError:
            with open(p, 'w+') as f:
                f.write(f"MODULE {mod}\nENDMODULE")
            self.write_array(name, li, path, type, subtype, mod)
        
    def write_var(self, name, li, type='VAR', subtype='robtarget', mod='arrs_mod', path=''):
        """
        Method for adding array into module of saved arrays
        NB! li (the values) must be formatted correctly (according to RAPID)
        NB! will overwrite variables
        NB! no variable name can be the postscript of an earlier variable, such a variable will be overwritten
        NB! SLOW code
        """
        p=pathlib.Path(path,f"{mod}.modx")
        try: 
            with open(p, 'r+') as f:
                lines = f.readlines()
                f.seek(0)
                for line in lines: #did not manage to do this in the while below
                    if f"{name}" not in line:
                        f.write(line)
                f.truncate()
                f.seek(0)
                pos, text = 0, ''
                while True: #SO magic
                    # save last line value and cursor position
                    prev_pos, pos = pos, f.tell()
                    prev_text, text = text, f.readline()
                    if text == '':
                        break
                f.seek(prev_pos, 0) # replace cursor to the last line
                if subtype in ['speeddata', 'zonedata']:
                    f.write(f"\t{type} {subtype} {name} := {str(li)};\n".replace("'",""))
                else:
                    f.write(f"\t{type} {subtype} {name} := {str(li)};\n")
                f.write(prev_text) # write old last line
        except OSError:
            with open(p, 'w+') as f:
                f.write(f"MODULE {mod}\nENDMODULE")
            self.write_var(name, li, path, type, subtype, mod)

    def load_module(self, name='arrs_mod', overwrite=True, path='$HOME/'):
        """"
        loads module from (RWS?) relative path
        NB! overwrites by default
        """
        self.toggle_mastership(1)
        if overwrite is True:
            resp = self.unload_module(name)
            if resp.status_code != 204:
                print(f'Unload module might have failed, status code {resp.status_code}, name {name}')
        resp = self.session.post(f'{self.base_url}/rw/rapid/tasks/{self.task}/loadmod', headers=dict(self.headers, **self.headers_json), data={'modulepath': f"{path}{name}.modx"}, auth=self.session.auth, verify=self.verify)
        self.toggle_mastership(0)
        return resp
    
    def unload_module(self, name='vca_mod'):
        """
        unloads module from self.task
        """
        resp = self.session.post(f'{self.base_url}/rw/rapid/tasks/{self.task}/unloadmod', headers=self.headers, data={'module': name}, auth=self.session.auth, verify=self.verify)
        return resp

    def set_rapid_variable(self, var, value, initval='true', module='arrs_mod'):
        """
        Sets the value of any RAPID variable.
        Unless the variable is of type 'num', 'value' has to be a string.
        Actually works somehow
        """
        #print(self.toggle_motors(0))
        #print(self.toggle_mastership(1))
        params = (
            ('initval', initval),
        )
        payload = {'value': value if isinstance(value, str) else str(value)}
        self.toggle_mastership(1)
        resp = self.session.post(f"{self.base_url}/rw/rapid/symbol/RAPID/{self.task}/{module}/{var}/data", verify=self.verify, data=payload, headers=self.headers, auth=self.session.auth, params=params)
        self.toggle_mastership(0)
        return resp

    def get_rapid_variable_properties(self, var):
        """
        Not useful atm
        Gets the properties of any RAPID variable.
        TODO: json parsing
        """
        resp = self.session.get(f"{self.base_url}/rw/rapid/symbol/RAPID/{self.task}/{self.module}/{var}/properties", verify=self.verify, headers=self.headers_json, auth=self.session.auth)
        return resp

    def get_rapid_variable(self, var, api=False, module="arrs_mod"):
        """
        Gets the value of any RAPID variable.
        """
        resp = self.session.get(f"{self.base_url}/rw/rapid/symbol/RAPID/{self.task}/{module}/{var}/data", verify=self.verify, headers=self.headers_json, auth=self.session.auth)
        if resp.status_code == 200:
            v = json.loads(resp.text)["state"][0]["value"]
            return (v, resp) if api else v
        else:
            print(f"Call for variable {var} might have failed\nstatus: {resp.status_code}\nbody: {resp.text}")
            return resp
    

    def wait_for_rapid(self, var='ready_flag'):
        """
        Waits for robot to complete RAPID instructions
        until boolean variable in RAPID is set to 'TRUE'.
        Default variable name is 'ready_flag', but others may be used.
        TODO:  matter?
        """
        while self.get_rapid_variable(var) == "FALSE" or self.is_running():
            time.sleep(0.1)
        self.set_rapid_variable('ready_flag', 'FALSE')


    def reset_pp(self):
        """
        Resets the program pointer to main procedure in RAPID.
        """
        self.toggle_mastership(1)
        resp = self.session.post(self.base_url + '/rw/rapid/execution/resetpp', auth=self.session.auth, verify=self.verify, headers=self.headers)
        self.toggle_mastership(0)
        return resp

    def toggle_mastership(self, tog = 0):
        """
        toggles write-permission (if allowed)
        Accepts 0 or 1 as input.
        """
        resp = self.session.post(self.base_url + '/rw/mastership/' + ['release', 'request'][tog], verify=self.verify, auth=self.session.auth, headers=self.headers)
        return resp

    def toggle_motors(self, tog=0):
        """
        Toggles the robot's motors.
        Accepts 0 or 1 as input.
        Operation mode has to be AUTO.
        """
        payload = {'ctrl-state': ['motoroff', 'motoron'][tog]}
        resp = self.session.post(self.base_url + "/rw/panel/ctrl-state", data=payload, headers=self.headers, auth=self.session.auth, verify=self.verify)
        return resp

    def start_RAPID(self):
        """
        Resets program pointer to main procedure in RAPID and starts RAPID execution.
        mastership:implicit very important
        Cannot run execution/start with mastership, but mastership is requred for /resetpp. weird.
        """
        self.reset_pp()
        params = (
            ('mastership', 'implicit'),
        )
        payload = {
            'regain': 'continue',
            'execmode': 'continue',
            'cycle': 'once',
            'condition': 'none',
            'stopatbp': 'disabled',
            'alltaskbytsp': 'true'
        }
        #self.toggle_motors(1)
        resp = self.session.post(self.base_url + "/rw/rapid/execution/start", data=payload, headers=self.headers, auth=self.session.auth, verify=self.verify, params=params)
        print(f"disconnecting client, status code: {self.disconnect_user().status_code}")
        return resp

    def stop_RAPID(self):
        """
        Stops RAPID execution.
        """

        data = {
        'stopmode': 'stop'
        }

        resp = self.session.post(f'{self.base_url}/rw/rapid/execution/stop', headers=self.headers, data=data, auth=self.session.auth, verify=self.verify)
        return resp

    def disconnect_user(self):
        data = {
            'client-type': '3'
        }

        resp = self.session.post(f"{self.base_url}/rw/infostream/disconnectclient", data=data, headers=self.headers, auth=self.session.auth, verify=self.verify)
        print(f"disconnectclient: {resp}")
        resp = self.session.post(f"{self.base_url}/rw/_istream/streamdisconnect", data=data, headers=self.headers, auth=self.session.auth, verify=self.verify)
        return resp

    def get_exec_state(self, api=False):
        """Gets the execution state of the controller.
        """

        resp = self.session.get(f"{self.base_url}/rw/rapid/execution", headers=self.headers_json, auth=self.session.auth, verify=self.verify)
        if resp.status_code == 200:
            v = json.loads(resp.text)["state"][0]["ctrlexecstate"]
            return (v, resp) if api else v
        else:
            print(f"Call might have failed\nstatus: {resp.status_code}\nbody: {resp.text}")
            return resp

    def is_running(self):
        """Checks the execution state of the controller and
        """
        return True if self.get_exec_state() == "running" else False
    
    def set_io_signal(self, IODevice = 'EN_Internal_Device', tog = 1, startbyte = '0', dataMask = 255):
        """Sets an IO signal in the controller
        """
        signalBits = ['01111111', '11111111'] #close / open

        payload = {'startbyte': startbyte, 'signaldata': signalBits[tog], 'datamask': dataMask}

        resp = self.session.post(self.base_url + "/rw/iosystem/devices/127.0.0.1/" + IODevice + "/set-inputdata", data=payload, headers=self.headers, verify=self.verify)
        return resp