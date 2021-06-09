""" Manage Versions in a python project
This will bump the file holding the current version (src/version.txt) as well as tag releases 
"""
import argparse
from pathlib import Path
from git import Repo
import sys

def parse_args():
    parser = argparse.ArgumentParser("Manage the project version including git tags.")
    parser.add_argument('--bump',
                        default='none',
                        const='none',
                        nargs='?',
                        choices=['major', 'minor', 'patch', 'none'],
                        help='Bump version of project')
    parser.add_argument('--version_file',
                        default='src/version.txt',
                        help='Path to version file')
    parser.add_argument("--tag",
                        dest="tag",
                        action="store_true",
                        default=False,
                        help="Create git tag")
    args = parser.parse_args()

    assert Path(args.version_file).exists()

    return args

def bump(bump_type:str='patch', version_file='src/version.txt'):
    with open('src/version.txt', 'r') as f:
        lines = f.readlines()
    version = ([int(line.split(' ')[1].strip()) for line in lines if line])

    # Increment the version, resetting minor/patches
    if bump_type == 'major':
        version[0] += 1
        version[1] = 0
        version[2] = 0
    if bump_type == 'minor':
        version[1] += 1
        version[2] = 0
    if bump_type == 'patch':
        version[2] += 1
    
    if bump_type != 'none':
        with open('src/version.txt', 'w') as f:
            f.write("MAJOR {}\nMINOR {}\nPATCH {}".format(*version))
    else:
        pass
        # print("Not incrementing version, bump was 'none'.\n")

    string_ints = [str(int_version) for int_version in version]
    version_str = ".".join(string_ints)

    return version_str

def create_tag(version_str, message="Tagged for release"):
    version_str = 'v{}'.format(version_str)
    repo = Repo('.')
    if repo.is_dirty():
        resp:str = input("Repo is dirty do you want to continue? [y/n]: ")
        if resp.lower() != 'y':
            print("Quiting...")
            sys.exit()
    hexsha = repo.head.commit.hexsha
    print("Creating new tag: {} ; commit: {}".format(version_str, hexsha))
    new_tag = repo.create_tag(version_str, message=message)
    return version_str
    
def main():
    args = parse_args()
    if args.tag:
        version_str = bump(bump_type='none', version_file=args.version_file)
        new_tag = create_tag(version_str)
        print("Don't forget to push the tags with:")
        print("\t git push origin {}  OR".format(new_tag))
        print("\t git push origin --tags")
    else:
        version_str = bump(bump_type=args.bump, version_file=args.version_file)
        print('Incremented version to {}'.format(version_str))
    



if __name__ == "__main__":
    main()