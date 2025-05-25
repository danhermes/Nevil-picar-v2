import dropbox
import os
from dotenv import load_dotenv
from helpers.utils import *
from dropbox.files import WriteMode

load_dotenv()

class LoadRAGfiles:
    def __init__(self):
        self.DB_ACCESS_TOKEN = os.getenv("DROPBOX_API_TOKEN")
        self.file_path = ""  # Path in Dropbox
        self.save_path = "" #Local path
        if not self.DB_ACCESS_TOKEN:
            gray_print("Error: No Dropbox access token found in environment 1")


    def get_files_from_Dropbox(self, file_type="general") -> str:
        if not self.DB_ACCESS_TOKEN:
            gray_print("Error: No Dropbox access token found in environment 2")
            return ""

        try:
            dbx = dropbox.Dropbox(self.DB_ACCESS_TOKEN)
            # Test the connection
            dbx.users_get_current_account()
            gray_print("Successfully connected to Dropbox")
        except dropbox.exceptions.AuthError:
            gray_print("Error: Invalid Dropbox access token")
            return ""
        except Exception as e:
            gray_print(f"Error connecting to Dropbox: {e}")
            return ""

        text = ""
        gray_print("Loading Topics...")
        
        # List all files and folders in root
        gray_print("Available paths in Dropbox:")
        try:
            entries = dbx.files_list_folder("").entries
            if not entries:
                gray_print("  No files found in Dropbox root")
                # gray_print("  Creating required folder and file...")
                # if not self.create_folder_and_file(dbx):
                #     return ""
            for entry in entries:
                gray_print(f"  {entry.path_display}")
                
            # Also try listing the RAgents folder specifically
            # try:
            #     ragents_entries = dbx.files_list_folder("/RAgents").entries
            #     gray_print("\nFiles in /RAgents folder:")
            #     for entry in ragents_entries:
            #         gray_print(f"  {entry.path_display}")
            # except dropbox.exceptions.ApiError as e:
            #     gray_print(f"  /RAgents folder not found or not accessible")
                #gray_print("  Creating required folder and file...")
                # if not self.create_folder_and_file(dbx):
                #     return ""
                
        except dropbox.exceptions.ApiError as e:
            gray_print(f"Error listing files: {e}")
            gray_print("Check if token has 'files.metadata.read' permission")
            return ""

        gray_print(f"File type: {file_type}")
        if file_type == "general":
            self.file_path = "/Nevil_general.txt"
        elif file_type == "fitness":
            self.file_path = "/weekly_fitness_plan.txt"
        elif file_type == "work":
            self.file_path = "/weekly_work_plan.txt"
        else:
            gray_print(f"Invalid file type: {file_type}")
            return ""
        gray_print(f"File path: {self.file_path}")

        # Try to download the file
        try:
            metadata, res = dbx.files_download(self.file_path)
            text = res.content.decode('utf-8')  # Convert bytes to string
            gray_print(f"Topics loaded from: {self.file_path}")
            
            # Optionally save locally
            with open(self.save_path, "w") as f:
                f.write(text)
                
        except dropbox.exceptions.ApiError as e:
            gray_print(f"Dropbox API Error downloading file: {e}")
            gray_print(f"Make sure {self.file_path} exists in your Dropbox")
            gray_print("Required permissions: files.content.read")
            return ""

        return text

        

    def create_folder_and_file(self, dbx):
        try:
            # Create RAgents folder
            try:
                dbx.files_create_folder_v2("/RAgents")
                gray_print("Created /RAgents folder")
            except dropbox.exceptions.ApiError as e:
                if not isinstance(e.error, dropbox.files.CreateFolderError.path_conflict):
                    raise e
                gray_print("/RAgents folder already exists")

            # Create initial d.txt if it doesn't exist
            initial_content = "Initial topics:\n- Example topic 1\n- Example topic 2"
            try:
                dbx.files_upload(
                    initial_content.encode(),
                    self.file_path,
                    mode=WriteMode.overwrite
                )
                gray_print(f"Created {self.file_path}")
            except Exception as e:
                gray_print(f"Error creating file: {e}")
                return False
            return True
        except Exception as e:
            gray_print(f"Error in create_folder_and_file: {e}")
            return False