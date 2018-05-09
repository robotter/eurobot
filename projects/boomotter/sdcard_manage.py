#!/usr/bin/env python3
import os
import re


class SdFolder:
    def __init__(self, path, track_num_length):
        self.track_num_length = track_num_length
        self.path = path
        self.reload()

    def reload(self):
        """Reload state from SD card"""

        self.extra_paths = []  # non-files, unsupported extensions
        self.bad_paths = []  # wrong basename format
        self.duplicates = []  # paths with duplicate track number

        self.tracks = {}  # {num: path}
        re_basename = re.compile(r"^\d{%d} - " % self.track_num_length)
        for p in os.listdir(self.path):
            subpath = os.path.join(self.path, p)
            if not os.path.isfile(subpath):
                self.extra_paths.append(p)
            elif not p.endswith('.mp3') and not p.endswith('.wav'):
                self.extra_paths.append(p)
            elif not re_basename.match(p):
                self.bad_paths.append(p)
            else:
                track_num = int(p[:self.track_num_length])
                if track_num in self.tracks:
                    self.duplicates.append(p)
                else:
                    self.tracks[track_num] = p

    def missing_numbers(self):
        if not self.tracks:
            return []
        max_num = max(self.tracks)
        return [i for i in range(1, max_num) if i not in self.tracks]

    def gen_free_track_numbers(self):
        if self.tracks:
            yield from self.missing_numbers()
            num = max(self.tracks) + 1
        else:
            num = 1
        while True:
            yield num
            num += 1

    def check(self):
        dirbase = os.path.basename(self.path)
        basename_format = '%s - track name.ext' % ('N' * self.track_num_length)

        if self.extra_paths:
            print("%s/ -- extra elements (%d), should be removed" % (dirbase, len(self.extra_paths)))
            for p in self.extra_paths:
                print("  %s/%s" % (dirbase, p))
            print()

        if self.bad_paths:
            print("%s/ -- bad file format (%d), should be renamed '%s'" % (dirbase, len(self.bad_paths), basename_format))
            for p in self.bad_paths:
                print("  %s/%s" % (dirbase, p))
            print()

        if self.duplicates:
            print("%s/ -- duplicate track numbers (%d), should be renamed to use a unique track number" % (dirbase, len(self.duplicates)))
            for p in self.duplicates:
                print("  %s/%s" % (dirbase, p))
            print()

        missing_numbers = self.missing_numbers()
        if missing_numbers:
            str_numbers = [str(i) for i in missing_numbers]
            print("%s/ -- discontinuous track numbers, missing: %s" % (dirbase, ' '.join(str_numbers)))

    def fix_tracks(self, dry_run=False):
        it_num = self.gen_free_track_numbers()
        dirbase = os.path.basename(self.path)
        to_rename = self.bad_paths + self.duplicates
        for p in to_rename:
            m = re.match(r"^\d\d+(.*)", p)
            if m:
                name = m.group(1)
            else:
                name = p
            new_subpath = "%0*d - %s" % (self.track_num_length, next(it_num), name.strip(' -.'))
            print("rename %s/%s --> %s/%s" % (dirbase, p, dirbase, new_subpath))
            if not dry_run:
                os.rename(os.path.join(self.path, p), os.path.join(self.path, new_subpath))


class SdCard:

    ignored_root_files = [
        'README.txt',
        'dfplayer_tracks.h',
        'sdcard_manage.py',
    ]

    folder_names = {
        '01': 'musics',
        '02': 'sounds',
    }

    def __init__(self, path):
        self.path = path
        self.reload()

    def reload(self):
        """Reload state from SD card"""

        self.mp3_folder = None
        self.num_folders = {}  # {num: files}
        self.extra_paths = []  # unexpected paths (including audio files)
        self.audio_files = []

        for p in os.listdir(self.path):
            if p in self.ignored_root_files:
                continue
            subpath = os.path.join(self.path, p)
            if p == 'mp3':
                self.mp3_folder = SdFolder(subpath, 4)
            elif re.match(r"^\d\d$", p):
                num = int(p)
                assert num not in self.num_folders
                self.num_folders[num] = SdFolder(subpath, 3)
            elif not os.path.isfile(subpath):
                self.extra_paths.append(p)
            elif not p.endswith('.mp3') and not p.endswith('.wav'):
                self.extra_paths.append(p)
            else:
                self.audio_files.append(p)

    def check(self):
        """Print suspicious elements"""

        if self.extra_paths:
            print("root -- extra elements (%d), should be removed" % len(self.extra_paths))
            for p in self.extra_paths:
                print("  %s" % p)
            print()

        if self.audio_files:
            print("root -- audio files (%d), should be moved to 'NN/' subdirectories" % len(self.audio_files))
            for p in self.audio_files:
                print("  %s" % p)
            print()

        if self.mp3_folder:
            self.mp3_folder.check()

        for folder in self.num_folders.values():
            folder.check()

    def fix_tracks(self, dry_run=False):
        if self.mp3_folder:
            self.mp3_folder.fix_tracks(dry_run)

        for folder in self.num_folders.values():
            folder.fix_tracks(dry_run)

    @staticmethod
    def basename_to_c_name(name):
        m = re.match(r"^\d+ - (.*)", os.path.splitext(name)[0])
        assert m is not None
        name = m.group(1).lower()
        name = re.sub(r"[- _.,()\[\]]+", '_', name)
        name = re.sub(r"[^a-z0-9_]", '', name)
        name = re.sub('_+', '_', name)
        return name.strip('_')

    def update_lists(self):
        if self.mp3_folder:
            mp3_enum = [(self.basename_to_c_name(p), num) for num, p in self.mp3_folder.tracks.items()]
        num_enum = []
        for fnum, folder in self.num_folders.items():
            folder_name = os.path.basename(folder.path)
            if folder_name in self.folder_names:
                folder_name = self.folder_names[folder_name]
            num_enum.extend(("%s_%s" % (folder_name, self.basename_to_c_name(p)), fnum * 0x100 + num) for num, p in folder.tracks.items())

        tracks_list_h = os.path.join(self.path, "dfplayer_tracks.h")
        with open(tracks_list_h, 'w') as f:
            f.write("typedef enum {\n")
            for s, v in sorted(mp3_enum):
                f.write("  TRACK_MP3_%s = %d,\n" % (s.upper(), v))
            f.write("} track_mp3_t;\n\n")

            f.write("typedef enum {\n")
            for s, v in sorted(num_enum):
                f.write("  TRACK_%s = %d,\n" % (s.upper(), v))
            f.write("} track_num_t;\n\n")
        print("generated %s" % tracks_list_h)



def command_update(parser, args):
    sdcard = SdCard(args.path)
    sdcard.fix_tracks(args.dry_run)
    sdcard.reload()
    sdcard.check()
    if not args.dry_run:
        sdcard.update_lists()

def command_check(parser, args):
    sdcard = SdCard(args.path)
    sdcard.check()

def command_fix(parser, args):
    sdcard = SdCard(args.path)
    sdcard.fix_tracks(args.dry_run)


def guess_sd_mountpoint():
    """Try to guess SD card mountpoint"""

    if os.name == 'nt':
        return None  # cannot guess on Windows

    diskname = "MP3"
    label_link = "/dev/disk/by-label/%s" % diskname
    if not os.path.islink(label_link):
        return None
    mp3_dev = os.path.realpath(label_link)
    for line in open("/proc/mounts"):
        mount_dev, mount_path, *_ = line.split()
        if mount_dev == mp3_dev:
            return mount_path
    return None


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Manage SD card files for Boomotter")
    parser.add_argument('-p', '--path', metavar='PATH',
                        help="path to SD card mountpoint (default: guessed)")
    subparsers = parser.add_subparsers(dest='command', help="command")

    subparser = subparsers.add_parser('update',
                                      help="fix track numbers, check, update track list files")
    subparser.add_argument('-n', '--dry-run', action='store_true',
                           help="print actions but don't actually modify anything")

    subparser = subparsers.add_parser('check',
                                      help="check SD card tree, print suspicious and invalid elements")

    subparser = subparsers.add_parser('fix',
                                      help="fix track numbers")
    subparser.add_argument('-n', '--dry-run', action='store_true',
                           help="print actions but don't actually modify anything")

    args = parser.parse_args()

    if not args.path:
        args.path = guess_sd_mountpoint()
        if args.path is None:
            parser.error("cannot guess SD card path, use --path")
    if not os.path.ismount(args.path):
        parser.error("path '%s' is not a mountpoint" % args.path)

    if not args.command:
        parser.error("missing command")

    globals()["command_" + args.command.replace('-', '_')](parser, args)

if __name__ == "__main__":
    main()

