from cmd import Cmd
import rerun as rr
import yaml
from pathlib import Path
import zarr

import render_zarr
import render_rosbag

class REPL(Cmd):

    def do_zarr(self, args):
        """Visualises a zarr file. Args: foldername(optional), episode_start(optional), episode_num(optional)
                foldername -> name of the zarr folder. Should be in zarr data dir spesified in config. (tab completion avaliable)
                episode_start -> idx of episode to visualise. Defaults to 0
                episode_num -> number of episodes to visualise. Defaults to 1
            Call \"zarr\" to see avaliable foldernames.
            """
        args = args.split()
        if len(args) == 0:
            # search 
            print(f"call zarr <foldername>. Avaliable folders are:")
            self._printAvaliableZarrs()
        else:
            try:
                episode_select = [0, 1]
                if len(args) > 1:
                    episode_select[0] = int(args[1])
                if len(args) > 2:
                    episode_select[1] = episode_select[0] + int(args[2])

                new_info = {"episode_select": episode_select}

                self._render("zarr", args[0], new_info)
            except:
                print("ERROR: Please input integers only. Type help.")

    def complete_zarr(self, text, line, begidx, endidx):
        if line.find(" ") > begidx:
            return []
        
        zarr_dir = Path(self._getConfig()["zarr_dir"])
        return [str(p) for p in zarr_dir.iterdir() if p.name.startswith(text)]
    
    def do_info_zarr(self, args):
        """Prints meta data of a zarr file. Args: foldername(optional)
                foldername -> name of the zarr folder. Should be in zarr data dir spesified in config. 
            Call \"zarr\" to see avaliable foldernames.
            """
        args = args.split()
        if len(args) == 0:
            print(f"call info_zarr <foldername>. Avaliable folders are:")
            self._printAvaliableZarrs()
        else:
            file = Path(self._getConfig()["zarr_dir"]) / args[0]
            if file.is_dir():
                print(zarr.open(str(file), mode="r").tree())
            else:
                print("ERROR: file doesnt exist")


    def do_rosbag(self, args):
        """Visualises a rosbag file. Args: foldername/number(optional)
                foldername/number ->  name or number of the bagfile directory (not the .db3 itself but the directory it is in) Should be in rosbag data dir spesified in config. (tab completion avaliable)
            call \"rosbag\" to see all avaliable bagfile directory names/numbers.
            """
        args = args.split()
        if len(args) == 0:
            # search 
            print(f"call rosbag <number | foldername>. Avaliable folders are:")
            self._printAvaliableRosbags()
        else:
            folder = args[0]
            if folder.isdigit():
                folder = int(folder)
                rosbag_dir = Path(self._getConfig()["rosbag_dir"])
                folders = list(sorted(rosbag_dir.iterdir()))
                if folder < 0 or folder >= len(folders):
                    print("ERROR: idx selection out of bounds.")
                    return
                dir = folders[folder]
                
            else:
                dir = Path(self._getConfig()["rosbag_dir"]) / folder
            
            for item in dir.iterdir():
                if item.suffix == self._getConfig()["rosbag_type"]:
                    path = dir.name + "/" + item.name

            self._render("rosbag", path)

    def complete_rosbag(self, text, line, begidx, endidx):
        if line.find(" ") > begidx:
            return []
        
        rosbag_dir = Path(self._getConfig()["zarr_dir"])
        return [str(p) for p in rosbag_dir.iterdir() if p.name.startswith(text)]
        
    def do_info_rosbag(self, args):
        """Prints meta data of a rosbag file. Args: foldername/number(optional)
                foldername/number ->  name or number of the bagfile directory (not the .db3 itself but the directory it is in) Should be in rosbag data dir spesified in config. 
            call \"info_rosbag\" to see all avaliable bagfile directory names/numbers."""
        args = args.split()
        if len(args) == 0:
            # search 
            print(f"call info_rosbag <number | foldername>. Avaliable folders are:")
            self._printAvaliableRosbags()
        else:
            folder = args[0]
            if folder.isdigit():
                folder = int(folder)
                rosbag_dir = Path(self._getConfig()["rosbag_dir"])
                folders = list(sorted(rosbag_dir.iterdir()))
                if folder < 0 or folder >= len(folders):
                    print("ERROR: idx selection out of bounds.")
                    return
                dir = folders[folder]
                
            else:
                dir = Path(self._getConfig()["rosbag_dir"]) / folder
            


            for item in dir.iterdir():
                if item.name == "metadata.yaml":
                    with item.open("r") as file:
                        data = yaml.safe_load(file)
                        print(data)
                        self._openDict("", data)
                    return
            print("Error: Metadata not found")

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

    def postcmd(self, stop, line):
        print()

    def _printAvaliableRosbags(self):
        rosbag_dir = Path(self._getConfig()["rosbag_dir"])
        print("\tidx:\tsize:\tfoldername")
        for idx, p in enumerate(sorted(rosbag_dir.iterdir())):
            if p.is_dir():
                size = "NIL"
                files = p.glob(f"*{self._getConfig()["rosbag_type"]}")
                for file in files:
                    size = file.stat().st_size/1e9
                    break
                print(f"\t{idx}\t{size:.2f}GB\t{p.name}")

    def _printAvaliableZarrs(self):
        zarr_dir = Path(self._getConfig()["zarr_dir"])
        for p in zarr_dir.iterdir():
            if p.is_dir():
                print(f"\t{p.name}")


    def _render(self, type, name, args={}):
        # get configs
        loaded_data = self._getConfig()
        for key, value in args.items():
            loaded_data[key] = value

        # find file
        loaded_data["datapath"] = Path(loaded_data[f"{type}_dir"]) / name
        if not loaded_data["datapath"].exists():
            print("ERROR: file does not exist.")
            return

        # set up rerun
        rr.init(loaded_data["application_id"])
        rr.spawn()

        # make blueprint nice
        rr.log_file_from_path(loaded_data["blueprint_path"])

        # run respective loaders
        if type == "zarr":
            render_zarr.render(loaded_data)
        elif type == "rosbag":
            render_rosbag.render(loaded_data)

    def _getConfig(self):
        location = Path(__file__).parent / "config.yaml"
        if location.is_file():
            with location.open('r') as file:
                return yaml.safe_load(file)
        else:
            print(f"ERROR: couldnt find \"config.yaml\" in the {Path(__file__).parent} directory. Please make sure it is present.")
            self.do_quit()

    def _openDict(self, prefix, datas):
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

if __name__ == '__main__':
    prompt = REPL()
    prompt.prompt = '> '
    prompt.intro = """
    Rerun Visualiser.
    Basic usage:
        help
        zarr 
        info_zarr
        rosbag 
        info_rosbag
        info_config
    """
    prompt.cmdloop()