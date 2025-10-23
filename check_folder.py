'''
  Tool to store, display, and check folder structures.

  # Store a folder structure to JSON
  python check_folder.py store ./my_folder
  
  # Show a folder structure (from folder or JSON)
  python check_folder.py show ./my_folder
  python check_folder.py show my_folder_tree.json
  
  # Check for missing files
  python check_folder.py check ./my_folder my_folder_tree.json

'''
import os
import json
import argparse
import sys
from pathlib import Path


class CheckFolder:
    """A class to store, display, and check folder structures."""
    
    def _build_tree(self, path):
        """Recursively build a tree structure of the given path."""
        path = Path(path)
        if not path.exists():
            raise ValueError(f"Path does not exist: {path}")
        
        result = {
            'name': path.name,
            'type': 'directory' if path.is_dir() else 'file',
        }
        
        if path.is_dir():
            children = []
            try:
                for item in sorted(path.iterdir()):
                    children.append(self._build_tree(item))
                if children:
                    result['children'] = children
            except PermissionError:
                pass
        
        return result
    
    def store(self, folder_name):
        """
        Store the tree structure of folder_name into a JSON file.
        
        Args:
            folder_name: Path to the folder to analyze
            
        Returns:
            The name of the generated JSON file
        """
        path = Path(folder_name)
        if not path.exists():
            raise ValueError(f"Folder does not exist: {folder_name}")
        
        tree = self._build_tree(folder_name)
        
        # Generate output filename
        output_file = f"{path.name}_tree.json"
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(tree, f, indent=2)
        
        print(f"Tree structure saved to {output_file}")
        return output_file
    
    def _print_tree(self, node, prefix="", is_last=True):
        """Recursively print the tree structure."""
        # Print the current node
        connector = "└── " if is_last else "├── "
        print(f"{prefix}{connector}{node['name']}")
        
        # If it's a directory with children, print them
        if node['type'] == 'directory' and 'children' in node:
            extension = "    " if is_last else "│   "
            children = node['children']
            for i, child in enumerate(children):
                is_last_child = (i == len(children) - 1)
                self._print_tree(child, prefix + extension, is_last_child)
    
    def show(self, folder_name):
        """
        Show the tree structure of a folder or JSON file (like Unix tree command).
        
        Args:
            folder_name: Path to a folder or JSON file
        """
        path = Path(folder_name)
        
        # Check if it's a JSON file
        if path.suffix == '.json' and path.exists():
            with open(path, 'r', encoding='utf-8') as f:
                tree = json.load(f)
        elif path.exists():
            # It's a folder, build the tree
            tree = self._build_tree(folder_name)
        else:
            raise ValueError(f"Path does not exist: {folder_name}")
        
        # Print the tree
        print(tree['name'])
        if tree['type'] == 'directory' and 'children' in tree:
            for i, child in enumerate(tree['children']):
                is_last = (i == len(tree['children']) - 1)
                self._print_tree(child, "", is_last)
    
    def _find_missing(self, actual_path, expected_node, parent_path=""):
        """Recursively find missing files/folders."""
        missing = []
        
        actual_path = Path(actual_path)
        # Build the full path for reporting
        full_path = os.path.join(parent_path, expected_node['name']) if parent_path else expected_node['name']
        
        if not actual_path.exists():
            missing.append(full_path)
            return missing
        
        # Check if type matches
        is_dir = actual_path.is_dir()
        expected_is_dir = expected_node['type'] == 'directory'
        
        if is_dir != expected_is_dir:
            missing.append(f"{full_path} (type mismatch: expected {expected_node['type']})")
            return missing
        
        # If it's a directory, check children
        if expected_is_dir and 'children' in expected_node:
            for child in expected_node['children']:
                child_actual_path = actual_path / child['name']
                missing.extend(self._find_missing(child_actual_path, child, full_path))
        
        return missing
    
    def check(self, folder_name, json_file):
        """
        Check for missing files in folder_name compared to json_file.
        Ignores additional files not in json_file.
        
        Args:
            folder_name: Path to the folder to check
            json_file: Path to the JSON file with expected structure
            
        Returns:
            List of missing files/folders
        """
        # Load the JSON file
        json_path = Path(json_file)
        if not json_path.exists():
            raise ValueError(f"JSON file does not exist: {json_file}")
        
        with open(json_path, 'r', encoding='utf-8') as f:
            expected_tree = json.load(f)
        
        # Check for missing files
        folder_path = Path(folder_name)
        
        # Find missing files starting from root
        missing = self._find_missing(folder_path, expected_tree)
        
        if missing:
            print(f"Missing files/folders ({len(missing)}):")
            for item in missing:
                print(f"  - {item}")
        else:
            print("✓ All expected files and folders are present!")
        
        return missing


def main():
    """Main function to handle command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Store, display, and check folder structures.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Store a folder structure to JSON
  python check_folder.py store ./my_folder
  
  # Show a folder structure (from folder or JSON)
  python check_folder.py show ./my_folder
  python check_folder.py show my_folder_tree.json
  
  # Check for missing files
  python check_folder.py check ./my_folder my_folder_tree.json
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    subparsers.required = True
    
    # Store command
    store_parser = subparsers.add_parser(
        'store',
        help='Store the tree structure of a folder into a JSON file'
    )
    store_parser.add_argument(
        'folder',
        help='Path to the folder to analyze'
    )
    
    # Show command
    show_parser = subparsers.add_parser(
        'show',
        help='Show the tree structure of a folder or JSON file'
    )
    show_parser.add_argument(
        'path',
        help='Path to a folder or JSON file'
    )
    
    # Check command
    check_parser = subparsers.add_parser(
        'check',
        help='Check for missing files in a folder compared to a JSON file'
    )
    check_parser.add_argument(
        'folder',
        help='Path to the folder to check'
    )
    check_parser.add_argument(
        'json_file',
        help='Path to the JSON file with expected structure'
    )
    
    args = parser.parse_args()
    
    checker = CheckFolder()
    
    try:
        if args.command == 'store':
            checker.store(args.folder)
        elif args.command == 'show':
            checker.show(args.path)
        elif args.command == 'check':
            missing = checker.check(args.folder, args.json_file)
            # Exit with non-zero status if files are missing
            if missing:
                sys.exit(1)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
