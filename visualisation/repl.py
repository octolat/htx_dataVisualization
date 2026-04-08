from cmd import Cmd
from dis import Instruction
import rerun as rr
import yaml
from pathlib import Path
import zarr

import render_zarr
import render_rosbag

class REPL(Cmd):
    def __init__(self):
        super().__init__()
        self._recordingCount = 0
        self.renderConfig = {
            "fileType": None,
            "filePath": None,
            "episode": None,
        }



    def do_zarr(self, args):
        # yo like make it n  and p type keys for both zarr and rosbag
        """Visualises a zarr file. 
            Args: <foldername | idx>, {episode_select}
                foldername -> name of the zarr folder. Should be in zarr data dir spesified in config. (tab completion avaliable)
                episode_select(optional) -> idx of episode to visualise. Defaults to 0
            Usage:
                zarr --> lists all avaliable folders in the zarr dir
                zarr <foldername | idx> --> renders the zarr, episode 0
                zarr <foldername | idx> <episode_select> --> renders the zarr, episode you provided
            """
        # get file from args
        args = args.split()
        instruction = "usage: zarr <foldername | idx> {episode_select}. Avaliable folders are:"
        file = self._fileSelection(args, instruction, "zarr")

        # render file
        if file != None:
            episode = 0
            if len(args) > 1:
                #check if its a int
                if args[1].isdigit():
                    episode = int(args[1])
                else:
                    print("ERROR: Please input integers only. Type help.")
                    return


            self.renderConfig["fileType"] = "zarr"
            self.renderConfig["filePath"] = file
            self.renderConfig["episode"] = episode

            self._render()
    
    def do_info_zarr(self, args):
        """Prints meta data of a zarr file. 
            Args: <foldername | idx> 
                foldername -> name of the zarr folder. Should be in zarr data dir spesified in config. 
            Usage:
                info_zarr --> lists all avaliable folders in the zarr dir
                zarr <foldername | idx> --> prints the metadata tree of the provided zarr
            """
        # get file from args
        args = args.split()
        instruction = "usage: info_zarr <foldername | idx>. Avaliable folders are:"
        file = self._fileSelection(args, instruction, "zarr")
        if file != None:
            print(zarr.open(str(file), mode="r").tree())

    def complete_zarr(self, text, line, begidx, endidx):
        return self._autocomplete(text, line, begidx, endidx, "zarr")
    
    def complete_zarr_info(self, text, line, begidx, endidx):
        return self._autocomplete(text, line, begidx, endidx, "zarr")






    def do_rosbag(self, args):
        """Visualises a rosbag file. 
            Args: <foldername | idx>, <bagfilename | episode_number>
                foldername -> name or idx of the group folder
                bagfilename -> name or idx of the spesific rosbag to render / episode number (idx == episode number)
            Usage:
                rosbag --> lists all avaliable folders in the rosbag dir
                rosbag <foldername | idx> --> lists all avaliable bagfiles
                rosbag <foldername | idx> <bagfilename | episode_number> --> renders the rosbag/episode you provided
            """
        
        # get file from args
        args = args.split()
        instruction = "usage: rosbag <foldername | idx> <bagfilename | episode_number>. Avaliable folders are:"
        file = self._fileSelection(args, instruction, "rosbag")

        # render file
        if file != None:
            episode = 0
            if len(args) < 2:
                # print bag file with their sizes
                self._printAvaliableFolders(file, additonal_info="size")
                print("usage: rosbag <foldername | idx> <bagfilename | episode_number>. Avaliable bagfiles above:")
            else:
                if args[1].isdigit():
                    episode = int(args[1])
                else:
                    # get episode from name (so cursed)
                    bagfile = self._getFileFromInput(file, args[1])
                    for idx, p in enumerate(list(sorted([f for f in file.iterdir() if f.is_dir()]))):
                        if p == bagfile:
                            episode = idx

                self.renderConfig["fileType"] = "rosbag"
                self.renderConfig["filePath"] = file
                self.renderConfig["episode"] = episode
                self._render()

    def do_info_rosbag(self, args):
        """Prints metadata of a rosbag file. 
            Args: <foldername | idx>, <bagfilename | episode_number>
                foldername -> name or idx of the group folder
                bagfilename -> name or idx of the spesific rosbag to show / episode number (idx == episode number)
            Usage:
                info_rosbag --> lists all avaliable folders in the rosbag dir
                info_rosbag <foldername | idx> --> lists all avaliable bagfiles and their sizes
                info_rosbag <foldername | idx> <bagfilename | episode_number> --> prints metadata of the rosbag you provided
            """
        
        # get file from args
        args = args.split()
        instruction = "usage: info_rosbag <foldername | idx>. Avaliable folders are:"
        file = self._fileSelection(args, instruction, "rosbag")
        if file != None:
            if len(args) < 2:
                # print bag file with their sizes
                self._printAvaliableFolders(file, additonal_info="size")
                print("usage: info_rosbag <foldername | idx> <bagfilename | idx> to see metadata. Avaliable bagfiles above:")
            else:
                # print metadata of a spesific bag file
                bagFolder = self._getFileFromInput(file, args[1])
                for item in bagFolder.iterdir():
                    if item.name == "metadata.yaml":
                        with item.open("r") as file:
                            data = yaml.safe_load(file)
                            print(data)
                            self._openDict("", data)
                        return
                print("Error: Metadata not found")

    def complete_rosbag(self, text, line, begidx, endidx):
        return self._autocomplete(text, line, begidx, endidx, "rosbag")
    
    def complete_rosbag_info(self, text, line, begidx, endidx):
        return self._autocomplete(text, line, begidx, endidx, "rosbag")

    def do_n(self, args):
        '''Prints the next episode.
            Usage:
                n'''
        if self.renderConfig["filePath"] == None:
            print("render something using zarr or rosbags first")
            return
        
        self.renderConfig["episode"] += 1
        self._render()

    def do_p(self, args):
        '''Prints the previous episode.
            Usage:
                p'''
        if self.renderConfig["filePath"] == None:
            print("render something using zarr or rosbags first")
            return
        
        self.renderConfig["episode"] -= 1
        self._render()


    def do_info_config(self, args):
        """Prints the current configuration of the visualizer. Updates are reflected immediately upon next visualising call"""
        loaded_data = self._getConfig()
        for key, value in loaded_data.items():
            print(f"-- {key}\t\t{value}")
            


    def do_quit(self, args):
        """Quits the program."""
        raise SystemExit
    
    def default(self, line):
        print("command not recognised. type help")

    def emptyline(self):
        pass

    def postcmd(self, stop, line):
        print()

    

    def _render(self):
        # check if anything in the current dict is non initalised
        if None in self.renderConfig.values():
            raise ValueError(f"render config dict is non initalised. dict = {self.renderConfig}")

        # check if episode range is sane
        if self.renderConfig["fileType"] == "zarr":
            try:
                max = zarr.open(str(self.renderConfig["filePath"]), mode="r")["meta"]["episode_ends"].shape[0]
            except:
                print("WARNING: This dataset lacks episode ends. This is probably a big issue, but assuming each episode is about 300 frames, and not going to check for episode bounds.")
                max = 100000
        elif self.renderConfig["fileType"] == "rosbag":
            max = sum([1 for f in self.renderConfig["filePath"].iterdir() if f.is_dir()])
        if self.renderConfig["episode"] < 0:
            print(f"ERROR: episode: {self.renderConfig["episode"]} is out of range. Resetting to {0}")
            self.renderConfig["episode"] = 0
        if self.renderConfig["episode"]+1 > max:
            print(f"ERROR: episode: {self.renderConfig["episode"]} is out of range. Resetting to {max-1}")
            self.renderConfig["episode"] = max-1

        # get config
        loaded_data = self._getConfig()

        # tick recording id (so that rerun registers this as a seperate episode)
        self._recordingCount += 1

        # set up rerun
        rr.init(loaded_data["application_id"], recording_id=str(self._recordingCount))
        rr.spawn()

        # make viewer nice
        group_name = self.renderConfig["filePath"].name
        rr.send_recording_name(f"{self.renderConfig["episode"]} - {self.renderConfig["fileType"]}:{group_name}")
        rr.log_file_from_path(loaded_data["blueprint_path"])

        # run respective logger
        if self.renderConfig["fileType"] == "zarr":
            loaded_data["episode_select"] = [self.renderConfig["episode"], self.renderConfig["episode"]+1]
            loaded_data["datapath"] = self.renderConfig["filePath"]
            render_zarr.render(loaded_data)

        elif self.renderConfig["fileType"] == "rosbag":
            # convert group folder to individual 
            loaded_data["datapath"] = self._getFileFromInput(self.renderConfig["filePath"], str(self.renderConfig["episode"]))
            render_rosbag.render(loaded_data)
       
    
    def _fileSelection(self, args, instruction, type):
        '''commander for selecting files '''
        if len(args) == 0:
            print(instruction)

            # print avaliable folders
            if type == "zarr":
                self._printAvaliableFolders(Path(self._getConfig()[f"{type}_dir"]), additonal_info="episode number")
            else:
                self._printAvaliableFolders(Path(self._getConfig()[f"{type}_dir"]))
            return None
        else:
            return self._getFileFromInput(Path(self._getConfig()[f"{type}_dir"]), args[0])
    
    def _getFileFromInput(self, dir, name):
        '''converts the args from a name or idx into a path object'''
        if name.isdigit():
            idx = int(name)
            folders = list(sorted([f for f in dir.iterdir() if f.is_dir()]))
            if idx < 0 or idx >= len(folders):
                print("ERROR: idx selection out of bounds.")
                return None
            return folders[idx]
        else:
            file = dir / name
            if file.exists():
                return file
            print("ERROR: file name does not exist.")
            return None
            
    def _printAvaliableFolders(self, data_dir, additonal_info=None):
        '''prints FOLDERs only'''
        if not additonal_info == None:
            print(f"\tidx:\t{additonal_info}:\tfoldername")
        else:
            print("\tidx:\tfoldername")
        for idx, p in enumerate(list(sorted([f for f in data_dir.iterdir() if f.is_dir()]))):
            if p.is_dir():
                if additonal_info == "size":
                    size = 0
                    files = p.glob(f"*{self._getConfig()["rosbag_type"]}")
                    for file in files:
                        size = file.stat().st_size/1e9
                        break
                    print(f"\t{idx}\t{size:.2f}GB\t{p.name}")
                elif additonal_info == "episode number":
                    try:
                        eps = zarr.open(str(p), mode="r")["meta"]["episode_ends"].shape[0]
                    except:
                        eps = -1
                    print(f"\t{idx}\t{eps}\t{p.name}")
                else:
                    print(f"\t{idx}:\t{p.name}")

    def _autocomplete(self, text, line, begidx, endidx, type):
        '''returns appropriate list for autocompleters'''
        
        if type == "zarr":
            if line[:begidx].count(" ") <= 1:
                # stage 1
                dir = Path(self._getConfig()["zarr_dir"])
                return [p.name for p in dir.iterdir() if p.name.startswith(text) and p.is_dir()]
            else:
                # stage 2
                return []
            
            
        elif type == "rosbag":
            if line[:begidx].count(" ") <= 1:
                # stage 1
                dir = Path(self._getConfig()["rosbag_dir"])
                return [p.name for p in dir.iterdir() if p.name.startswith(text) and p.is_dir()]
            elif line[:begidx].count(" ") <= 2:
                # stage 2
                dir = self._getFileFromInput(Path(self._getConfig()["rosbag_dir"]), line.split(" ")[-2])
                if dir == None:
                    return [] 
                return [p.name for p in dir.iterdir() if p.name.startswith(text) and p.is_dir()]
            else:
                return []
    

    def _getConfig(self):
        location = Path(__file__).parent / "config.yaml"
        if location.is_file():
            with location.open('r') as file:
                return yaml.safe_load(file)
        else:
            print(f"ERROR: couldnt find \"config.yaml\" in the {Path(__file__).parent} directory. Please make sure it is present.")
            self.do_quit("")

    def _openDict(self, prefix, datas):
        ''' recursively print a dict, mostly for a rosbag metadata.yaml dict tbh '''
        seperator = "  "

        if not isinstance(datas, list):
            datas = [datas]

        for data in datas:
            if isinstance(data, str):
                print(f"{prefix}{data}")
                continue
            for key, value in data.items():
                if isinstance(value, dict) or isinstance(value, list):
                    print(f"{prefix}{key}: ")
                    self._openDict(prefix+seperator, value)
                else:
                    if isinstance(value, str) and value.find("\n") >= 0:
                        print(f"{prefix}{key}: ")
                        for it in value[3:].split("\n  "):
                            print(f"{prefix+seperator}{it}")

                    else:
                        print(f"{prefix}{key}: {value}")

    def cmdloop(self, intro=None):
        """Repeatedly issue a prompt, accept input, parse an initial prefix
        off the received input, and dispatch to action methods, passing them
        the remainder of the line as argument.

        """

        self.preloop()
        if self.use_rawinput and self.completekey:
            try:
                import readline
                self.old_completer = readline.get_completer()
                readline.set_completer(self.complete)
                if readline.backend == "editline":
                    if self.completekey == 'tab':
                        # libedit uses "^I" instead of "tab"
                        command_string = "bind ^I rl_complete"
                    else:
                        command_string = f"bind {self.completekey} rl_complete"
                else:
                    command_string = f"{self.completekey}: complete"
                readline.parse_and_bind(command_string)
                # make it not take _ as a delim
                old_delims = readline.get_completer_delims() # <-
                readline.set_completer_delims(old_delims.replace('-', '')) # <-
            except ImportError:
                pass
        try:
            if intro is not None:
                self.intro = intro
            if self.intro:
                self.stdout.write(str(self.intro)+"\n")
            stop = None
            while not stop:
                if self.cmdqueue:
                    line = self.cmdqueue.pop(0)
                else:
                    if self.use_rawinput:
                        try:
                            line = input(self.prompt)
                        except EOFError:
                            line = 'EOF'
                    else:
                        self.stdout.write(self.prompt)
                        self.stdout.flush()
                        line = self.stdin.readline()
                        if not len(line):
                            line = 'EOF'
                        else:
                            line = line.rstrip('\r\n')
                line = self.precmd(line)
                stop = self.onecmd(line)
                stop = self.postcmd(stop, line)
            self.postloop()
        finally:
            if self.use_rawinput and self.completekey:
                try:
                    import readline
                    readline.set_completer(self.old_completer)
                except ImportError:
                    pass

if __name__ == '__main__':
    prompt = REPL()
    prompt.prompt = '> '
    prompt.intro = """
    Rerun Visualiser.
    Basic usage:

        help
        
        zarr <foldername | idx>, {episode_select}
        rosbag <foldername | idx>, <bagfilename | episode_number>

        info_zarr <foldername | idx> 
        info_rosbag <foldername | idx>, <bagfilename | episode_number>

    autocomplete avaliable.

    !!!
    Update --> you can now type "n" (for next) and "p" (for previous) to step through episodes. -siewling 
    !!!
    """
    
    prompt.cmdloop()
    # print(prompt._autocomplete("", "rosbag ", 7, 7, "rosbag"))
