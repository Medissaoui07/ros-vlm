import yaml
import os
from pathlib import Path

class PromptLoader:
    def __init__(self, config_path=None):
        if config_path is None:
            # Default to package config directory
            config_path = '/home/ken2/ros2_ws/src/ros_vlm/config/prompts.yaml'
        
        with open(config_path, 'r') as f:
            self.prompts = yaml.safe_load(f)
    
    def get_system_prompt(self, vision_context):
        """Combines base prompt with vision context"""
        base = self.prompts['system']['base']
        vision = self.prompts['system']['vision_context'].format(
            vision=vision_context
        )
        return f"{base}\n{vision}"
    
    def get_rate_limit_message(self, wait_time):
        """Gets rate limit message with wait time"""
        return self.prompts['system']['rate_limit'].format(
            wait_time=wait_time
        )