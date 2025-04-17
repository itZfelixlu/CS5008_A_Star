import pygame
import json
import time
import random
import os
import imageio
import numpy as np
from datetime import datetime
from PIL import Image
import io
import threading

# Initialize Pygame
pygame.init()

# Constants
WINDOW_SIZE = (1400, 800)  # Increased width to accommodate buttons on the right
GRID_SIZE = 50  # Reduced from 100 to make cells larger
CELL_SIZE = 12  # Increased from 6 to make cells larger
MARGIN = 50
BUTTON_HEIGHT = 40
BUTTON_WIDTH = 200
BUTTON_MARGIN = 20
RIGHT_PANEL_WIDTH = 250  # Width of the right panel for buttons

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
BROWN = (139, 69, 19)  # For slopes

# Create the window
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption("A* Algorithm Visualization")

class Button:
    def __init__(self, x, y, width, height, text, color=GRAY, hover_color=(180, 180, 180), action=None):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.font = pygame.font.Font(None, 24)
        self.action = action
        
    def draw(self, surface):
        color = self.hover_color if self.is_hovered() else self.color
        pygame.draw.rect(surface, color, self.rect)
        pygame.draw.rect(surface, BLACK, self.rect, 2)  # Border
        
        text_surface = self.font.render(self.text, True, BLACK)
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)
        
    def is_hovered(self):
        return self.rect.collidepoint(pygame.mouse.get_pos())
    
    def is_clicked(self, event):
        return event.type == pygame.MOUSEBUTTONDOWN and self.is_hovered()

class Grid:
    def __init__(self, width, height, cell_size):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self.start = (0, 0)
        self.end = (width-1, height-1)
        self.path = []
        self.explored_nodes = []
        self.current_animation_index = 0
        self.animation_speed = 5  # Increased from 1 to 5 for faster animation
        self.animation_delay = 0
        self.is_animating = False
        self.current_algorithm = None
        self.current_grid_type = None
        self.slope_regions = []  # Store slope regions for visualization
        self.show_path = False  # Flag to control when to show the path
        self.meeting_point = None  # Store the meeting point for bidirectional A*
        self.forward_path = []  # Store forward path
        self.backward_path = []  # Store backward path
        self.frames = []  # Store frames for GIF creation
        self.frame_skip = 1  # Only capture every nth frame
        self.frame_counter = 0
        self.max_frames = 1000  # Maximum number of frames to store
        self.is_saving = False
        self.save_thread = None
        
    def load_simple_grid(self):
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.start = (0, 0)
        self.end = (self.width-1, self.height-1)
        self.path = []
        self.explored_nodes = []
        self.current_animation_index = 0
        self.current_grid_type = "simple"
        self.slope_regions = []  # Clear slope regions
        self.show_path = False
        self.frames = []  # Clear frames when loading new grid
        self.frame_counter = 0
        # Adjust frame skip based on grid complexity
        if len(self.explored_nodes) > 1000:
            self.frame_skip = max(1, len(self.explored_nodes) // 500)  # Target ~500 frames
        else:
            self.frame_skip = 1
        
    def load_complicated_grid(self):
        # Load the grid setup from the JSON file
        try:
            with open('result_complicated.json', 'r') as f:
                data = json.load(f)
                grid_setup = data['grid_setup']
                
                # Clear the grid
                self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
                self.start = (0, 0)
                self.end = (self.width-1, self.height-1)
                self.path = []
                self.explored_nodes = []
                self.current_animation_index = 0
                self.current_grid_type = "complicated"
                
                # Add obstacles from the JSON file
                # Use a set to avoid duplicate obstacles
                obstacle_set = set()
                for obstacle in grid_setup['obstacles']:
                    x, y = obstacle
                    if 0 <= x < self.width and 0 <= y < self.height:
                        obstacle_set.add((x, y))
                
                # Add unique obstacles to the grid
                for x, y in obstacle_set:
                    self.grid[y][x] = 1  # Mark as obstacle
                
                # Add slope regions from the JSON file
                for region in grid_setup['slope_regions']:
                    x_min, y_min, x_max, y_max = region
                    # Remove the +1 to match C program's region size
                    for y in range(y_min, y_max):
                        for x in range(x_min, x_max):
                            if 0 <= x < self.width and 0 <= y < self.height:
                                if self.grid[y][x] != 1:  # Don't overwrite obstacles
                                    self.grid[y][x] = 2  # Mark as slope
        except Exception as e:
            print(f"Error loading complicated grid: {e}")
        
        self.frames = []  # Clear frames when loading new grid
        self.frame_counter = 0
        # Adjust frame skip based on grid complexity
        if len(self.explored_nodes) > 1000:
            self.frame_skip = max(1, len(self.explored_nodes) // 500)  # Target ~500 frames
        else:
            self.frame_skip = 1
        
    def load_results(self, algorithm, grid_type):
        filename = f"result_{grid_type}.json"
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
                algorithm_data = data[algorithm]
                self.path = algorithm_data['path']
                self.explored_nodes = algorithm_data['explored_nodes']
                self.current_animation_index = 0
                self.is_animating = True
                self.show_path = False  # Reset path display
                self.current_algorithm = algorithm
                self.current_grid_type = grid_type
                self.frames = []  # Clear any existing frames
                
                # For bidirectional A*, find the meeting point
                if algorithm == "bidirectional_a_star":
                    # Find the first node that appears in both forward and backward paths
                    forward_nodes = set()
                    backward_nodes = set()
                    meeting_index = -1
                    
                    for i, node in enumerate(self.explored_nodes):
                        node_key = (node['x'], node['y'])
                        if node_key in forward_nodes:
                            backward_nodes.add(node_key)
                            if meeting_index == -1:
                                meeting_index = i
                                self.meeting_point = node
                        else:
                            forward_nodes.add(node_key)
                            
                # Start capturing frames immediately
                if not self.is_saving:
                    frame = pygame.surfarray.array3d(pygame.display.get_surface())
                    frame = np.transpose(frame, (1, 0, 2)).astype(np.uint8)
                    self.frames.append(frame)
        except Exception as e:
            print(f"Error loading {filename}: {e}")
        
    def update_animation(self):
        if self.is_animating and self.current_animation_index < len(self.explored_nodes):
            # Update animation index with increased speed
            self.current_animation_index = min(self.current_animation_index + self.animation_speed, 
                                             len(self.explored_nodes))
            
            if self.current_animation_index >= len(self.explored_nodes):
                self.is_animating = False
                self.show_path = True  # Enable path display
                
    def save_animation_gif(self, filename=None):
        if not self.frames:
            print("No frames to save!")
            return
            
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"animation_{timestamp}.gif"
            
        print(f"Processing {len(self.frames)} frames...")
        
        # Start saving in a separate thread to avoid freezing the UI
        def save_thread():
            self.is_saving = True
            try:
                # Save as GIF with high quality settings and higher frame rate
                imageio.mimsave(
                    filename, 
                    self.frames, 
                    duration=0.016,  # ~60 FPS (1/60 seconds per frame)
                    quantizer='nq',  # Neural quantization for better quality
                    fps=60,  # 60 frames per second
                    loop=0,  # Loop infinitely
                    subrectangles=True  # Optimize by only storing changed parts
                )
                print(f"Animation saved as {filename}")
            except Exception as e:
                print(f"Error saving animation: {e}")
            finally:
                self.is_saving = False
                self.frames = []  # Clear frames after saving
                
        self.save_thread = threading.Thread(target=save_thread)
        self.save_thread.start()
        
    def draw(self, screen):
        # Draw grid background
        for y in range(self.height):
            for x in range(self.width):
                rect = pygame.Rect(x * self.cell_size, y * self.cell_size, 
                                 self.cell_size, self.cell_size)
                if self.grid[y][x] == 0:  # Normal terrain
                    pygame.draw.rect(screen, WHITE, rect)
                elif self.grid[y][x] == 1:  # Obstacle
                    pygame.draw.rect(screen, BLACK, rect)
                elif self.grid[y][x] == 2:  # Slope
                    pygame.draw.rect(screen, BROWN, rect)
        
        # Draw explored nodes up to current animation index
        for i in range(self.current_animation_index):
            node = self.explored_nodes[i]
            x, y = node['x'], node['y']
            if self.grid[y][x] != 1:  # Don't draw on obstacles
                pygame.draw.rect(screen, BLUE, (x * self.cell_size, y * self.cell_size, 
                                              self.cell_size, self.cell_size))
        
        # Draw final path in green when animation is complete
        if self.path and (self.show_path or not self.is_animating):
            for node in self.path:
                x, y = node['x'], node['y']
                if self.grid[y][x] != 1:  # Don't draw on obstacles
                    pygame.draw.rect(screen, GREEN, (x * self.cell_size, y * self.cell_size, 
                                                   self.cell_size, self.cell_size))
        
        # Draw start and end points
        start_rect = pygame.Rect(self.start[0] * self.cell_size, 
                               self.start[1] * self.cell_size, 
                               self.cell_size, self.cell_size)
        end_rect = pygame.Rect(self.end[0] * self.cell_size, 
                             self.end[1] * self.cell_size, 
                             self.cell_size, self.cell_size)
        pygame.draw.rect(screen, RED, start_rect)
        pygame.draw.rect(screen, RED, end_rect)

        # Draw grid lines
        for x in range(self.width + 1):
            pygame.draw.line(screen, GRAY, (x * self.cell_size, 0), 
                           (x * self.cell_size, self.height * self.cell_size))
        for y in range(self.height + 1):
            pygame.draw.line(screen, GRAY, (0, y * self.cell_size), 
                           (self.width * self.cell_size, y * self.cell_size))

        # Draw meeting point for bidirectional A*
        if self.meeting_point and self.current_animation_index > 0:
            x = self.meeting_point['x'] * self.cell_size
            y = self.meeting_point['y'] * self.cell_size
            pygame.draw.rect(screen, RED, (x, y, self.cell_size, self.cell_size))

        # Capture frame after everything is drawn
        if not self.is_saving:
            frame = pygame.surfarray.array3d(pygame.display.get_surface())
            frame = np.transpose(frame, (1, 0, 2)).astype(np.uint8)
            self.frames.append(frame)
            
            # If this is the final state, capture additional frames
            if not self.is_animating and self.show_path:
                for _ in range(20):  # Capture 20 frames of the final state
                    frame = pygame.surfarray.array3d(pygame.display.get_surface())
                    frame = np.transpose(frame, (1, 0, 2)).astype(np.uint8)
                    self.frames.append(frame)

def main():
    clock = pygame.time.Clock()
    running = True
    
    # Calculate button positions for the right panel
    button_start_x = WINDOW_SIZE[0] - RIGHT_PANEL_WIDTH + (RIGHT_PANEL_WIDTH - BUTTON_WIDTH) // 2
    button_start_y = MARGIN
    
    # Create grid selection buttons
    grid_buttons = [
        Button(button_start_x, button_start_y, BUTTON_WIDTH, BUTTON_HEIGHT, "Simple Grid"),
        Button(button_start_x, button_start_y + (BUTTON_HEIGHT + BUTTON_MARGIN), BUTTON_WIDTH, BUTTON_HEIGHT, "Complicated Grid"),
        Button(button_start_x, button_start_y + 3 * (BUTTON_HEIGHT + BUTTON_MARGIN), BUTTON_WIDTH, BUTTON_HEIGHT, "Regular A*"),
        Button(button_start_x, button_start_y + 4 * (BUTTON_HEIGHT + BUTTON_MARGIN), BUTTON_WIDTH, BUTTON_HEIGHT, "Bidirectional A*"),
        Button(button_start_x, button_start_y + 5 * (BUTTON_HEIGHT + BUTTON_MARGIN), BUTTON_WIDTH, BUTTON_HEIGHT, "Dynamic A*"),
        Button(button_start_x, button_start_y + 6 * (BUTTON_HEIGHT + BUTTON_MARGIN), BUTTON_WIDTH, BUTTON_HEIGHT, "Save Animation GIF")
    ]
    
    grid = Grid(GRID_SIZE, GRID_SIZE, CELL_SIZE)
    last_update = 0
    
    while running:
        current_time = time.time()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
            # Handle button clicks
            if event.type == pygame.MOUSEBUTTONDOWN:
                for i, button in enumerate(grid_buttons):
                    if button.is_hovered():
                        if i == 0:  # Simple Grid
                            grid.load_simple_grid()
                        elif i == 1:  # Complicated Grid
                            grid.load_complicated_grid()
                        elif i == 2:  # Regular A*
                            if grid.current_grid_type:
                                grid.load_results("regular_a_star", grid.current_grid_type)
                        elif i == 3:  # Bidirectional A*
                            if grid.current_grid_type:
                                grid.load_results("bidirectional_a_star", grid.current_grid_type)
                        elif i == 4:  # Dynamic A*
                            if grid.current_grid_type:
                                grid.load_results("dynamic_a_star", grid.current_grid_type)
                        elif i == 5:  # Save Animation GIF
                            grid.save_animation_gif()
        
        # Update animation more frequently
        if current_time - last_update > 0.05:  # Reduced from 0.1 to 0.05 seconds
            grid.update_animation()
            last_update = current_time
        
        # Draw everything
        screen.fill(WHITE)
        
        # Draw a separator line between grid and buttons
        pygame.draw.line(screen, GRAY, 
                        (WINDOW_SIZE[0] - RIGHT_PANEL_WIDTH, 0),
                        (WINDOW_SIZE[0] - RIGHT_PANEL_WIDTH, WINDOW_SIZE[1]), 2)
        
        # Draw a separator line between grid and algorithm buttons
        separator_y = button_start_y + 2.5 * (BUTTON_HEIGHT + BUTTON_MARGIN)
        pygame.draw.line(screen, GRAY,
                        (WINDOW_SIZE[0] - RIGHT_PANEL_WIDTH, separator_y),
                        (WINDOW_SIZE[0], separator_y), 2)
        
        grid.draw(screen)
        for button in grid_buttons:
            button.draw(screen)
            
        pygame.display.flip()
        clock.tick(60)
        
    pygame.quit()

if __name__ == "__main__":
    main() 