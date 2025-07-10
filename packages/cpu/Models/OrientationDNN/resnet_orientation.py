import numpy as np
import torch.nn as nn
import torch
import torchvision.models as models
import torchvision.transforms as transforms
from torch.utils.data import Dataset, DataLoader
from PIL import Image
import pandas as pd
import os


MODEL_PATH = "regression_resnet_101_loss_huber_lr_5e-5_e_200_patience_8_batch_32_ignoring_front.pth"
CONFIDENCE_THRESHOLD = 0.12

class OrientationDataset(Dataset):
    def __init__(self, image_base_dir="./prepared_images_602", label_base_dir="./prepared_labels_602", split="train", confidence_threshold=CONFIDENCE_THRESHOLD):
        print(f"Initializing class to manage Orientation Dataset for {split} split")
        
        #directory paths for the specified split
        self.image_dir = os.path.join(image_base_dir, split)
        self.label_dir = os.path.join(label_base_dir, split)
        
        self.confidence_threshold = confidence_threshold
        print(f"Using confidence threshold: {self.confidence_threshold}")

        #retrieve all images
        all_image_filenames = [os.path.splitext(f)[0] for f in os.listdir(self.image_dir) if f.endswith((".jpg", ".png"))]
        
        #If confidence under threshold not consider in dataset -> Done at initalization to save process time
        self.image_filenames = []
        for image_name in all_image_filenames:
            label_path = os.path.join(self.label_dir, image_name + ".txt")
            try:
                with open(label_path, "r") as f:
                    lines = f.readlines()
                confidence_line = lines[3]
                confidence = float(confidence_line.split(":")[1].strip())
                
                if confidence >= self.confidence_threshold:
                    self.image_filenames.append(image_name)
            except (IndexError, FileNotFoundError, ValueError) as e:
                print(f"Error processing {image_name}: {e}")
                continue
        
        print(f"Loaded {len(self.image_filenames)} images with confidence >= {self.confidence_threshold} (out of {len(all_image_filenames)} total)")
        
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),  # Needed to be resized to fit ResNet input size
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Preprocessing necessary for Resnet     
        ])
        
    def __len__(self):
        return len(self.image_filenames)

    def __getitem__(self, idx):
        #Get image and label paths
        image_name = self.image_filenames[idx]
        image_path = os.path.join(self.image_dir, image_name + ".png")
        label_path = os.path.join(self.label_dir, image_name + ".txt")

        
        image = Image.open(image_path).convert("RGB")
        image = self.transform(image) #transform to tensor resnet shape

        with open(label_path, "r") as f:
            lines = f.readlines()
    
        #vector info from the .txt file
        vector_line = lines[2]  # Third line contains the vector
        x, y = map(float, vector_line.split(":")[1].strip().strip("()").split(","))

        # Convert to tensor
        orientation = torch.tensor([x, y], dtype=torch.float32)

        return image, orientation
class OrientationResnet(nn.Module):
    def __init__(self, pretrained=True):
        super(OrientationResnet, self).__init__()
        
        # Test with Resnet18 (smaller backbone)
        base_model = models.resnet101(pretrained=pretrained)
        self.withouthLastLayer = nn.Sequential(*list(base_model.children())[:-2])  # Remove last pooling and Connected layer
        
        self.customAvgPool = nn.AdaptiveAvgPool2d((1, 1))
        
        self.regression = nn.Sequential(
            nn.Linear(2048, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, 2)  #(x,y) output
        )

    def forward(self, image):
        # Applied sequence declared at constructor
        image = self.withouthLastLayer(image)
        image = self.customAvgPool(image)
        image = torch.flatten(image, 1)
        
        projection = self.regression(image)
        
        return projection

class TrainModel:
    def __init__(self, device="cpu" if torch.cuda.is_available() else "cpu"):
        self.device = torch.device(device)
        self.model = OrientationResnet(pretrained=True).to(self.device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=0.00002)
        self.huber_loss = torch.nn.HuberLoss(delta=1.0)

    def calculate_angular_loss(self, pred_vectors, target_vectors):
        pred_norm = torch.nn.functional.normalize(pred_vectors, p=2, dim=1)
        target_norm = torch.nn.functional.normalize(target_vectors, p=2, dim=1)
    
        cos_sim = torch.sum(pred_norm * target_norm, dim=1)  # Cosine similarity
        angular_loss = 1 - cos_sim

        return angular_loss.mean()
        
    def train_step(self, images, target_vectors):
        images = images.to(self.device)
        target_vectors = target_vectors.to(self.device)
        
        # Forward pass
        predicted_vectors = self.model(images)
        
        # Calculate loss
        #angular_loss = self.calculate_angular_loss(predicted_vectors, target_vectors)
        
        # Normalize loss based on max value that can achieve
        #normalized_angular_loss = angular_loss / 2.0
        
        # Total Loss
        total_loss = self.huber_loss(predicted_vectors, target_vectors)
        
        # Backpropagation
        self.optimizer.zero_grad()
        total_loss.backward()
        self.optimizer.step()        

        return total_loss

def training_model(model, train_loader, val_loader, epochs, patience, model_path=MODEL_PATH):
    best_val_loss = float('inf')
    patience_counter = 0
    
    for epoch in range(epochs):
        model.model.train()
        total_train_loss = 0

        for images, target_vectors in train_loader:
            total_loss = model.train_step(images, target_vectors)
            total_train_loss += total_loss.item()

        avg_train_loss = total_train_loss / len(train_loader)
        print(f"Epoch [{epoch+1}/{epochs}], Train Loss: {avg_train_loss:.4f}")
    
        # Validation step
        model.model.eval()
        total_val_loss = 0
        with torch.no_grad():
            for images, target_vectors in val_loader:
                images = images.to(model.device)
                target_vectors = target_vectors.to(model.device)

                predicted_vectors = model.model(images)
                #angular_loss = model.calculate_angular_loss(predicted_vectors, target_vectors)
                #normalized_angular_loss = angular_loss / 2.0
                angular_loss = model.huber_loss(predicted_vectors, target_vectors)
                
                total_loss = angular_loss
                total_val_loss += total_loss.item()

        avg_val_loss = total_val_loss / len(val_loader)
        print(f"Epoch [{epoch+1}/{epochs}], Val Loss: {avg_val_loss:.4f}")
        
        # Check if validation loss improved
        if avg_val_loss < best_val_loss:
            print(f"Validation loss improved from {best_val_loss:.4f} to {avg_val_loss:.4f}, saving model...")
            best_val_loss = avg_val_loss
            torch.save(model.model.state_dict(), model_path)
            patience_counter = 0
        else:
            patience_counter += 1
            print(f"Validation loss did not improve. Patience: {patience_counter}/{patience}")
            
            if patience_counter >= patience:
                print(f"Early stopping triggered after {epoch+1} epochs!")
                break
    
    print(f"Training completed. Best validation loss: {best_val_loss:.4f}")

def train_model_and_save_it(image_base_dir="./prepared_images_602", label_base_dir="./prepared_labels_602", epochs=200, patience=8):
    # Initialize datasets for each split
    train_dataset = OrientationDataset(
        image_base_dir=image_base_dir, 
        label_base_dir=label_base_dir, 
        split="train"
    )
    
    val_dataset = OrientationDataset(
        image_base_dir=image_base_dir, 
        label_base_dir=label_base_dir, 
        split="validation" 
    )
    
    test_dataset = OrientationDataset(
        image_base_dir=image_base_dir, 
        label_base_dir=label_base_dir, 
        split="test"
    )

    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)
    test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)

    # Initialize model
    model = TrainModel()
    
    # Train the model with early stopping
    model_path = MODEL_PATH
    training_model(model, train_loader, val_loader, epochs=epochs, patience=patience, model_path=model_path)
    
    # Evaluate on test set
    model.model.load_state_dict(torch.load(model_path))
    model.model.eval()
    
    total_test_loss = 0
    with torch.no_grad():
        for images, target_vectors in test_loader:
            images = images.to(model.device)
            target_vectors = target_vectors.to(model.device)
            
            predicted_vectors = model.model(images)
            angular_loss = model.huber_loss(predicted_vectors, target_vectors)
            #normalized_angular_loss = angular_loss / 2.0
            
            total_loss = angular_loss
            total_test_loss += total_loss.item()
    
    avg_test_loss = total_test_loss / len(test_loader)
    print(f"Test Loss: {avg_test_loss:.4f}")

def calculate_angle_between_vectors(v1, v2):
    """Calculate the angle between two vectors in degrees."""
    # Normalize vectors
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    
    # Calculate dot product
    dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
    
    # Calculate angle in radians and convert to degrees
    angle_rad = np.arccos(dot_product)
    angle_deg = angle_rad * 180 / np.pi
    
    return angle_deg

def calculate_r2_score(pred_vectors, target_vectors):
    """
    Calculate the r2 score for vector prediction.
    """
    mean_target = np.mean(target_vectors, axis=0)
    ss_total = np.sum((target_vectors - mean_target) ** 2)
    
    ss_residual = np.sum((target_vectors - pred_vectors) ** 2)
    r2 = 1 - (ss_residual / ss_total)
    
    return r2

def test_trained_model(image_base_dir, label_base_dir, model_path):
    """
    Test the trained model on the test dataset and calculate metrics:
    - Mean angular error
    - Accuracy at 22.5° threshold
    - Accuracy at 45° threshold
    
    Args:
        image_base_dir: Base directory for test images
        label_base_dir: Base directory for test labels
        test_loader: Optional pre-created test data loader
        model_path: Path to the trained model weights
    """
    device = torch.device("cpu" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Load model
    model = OrientationResnet(pretrained=False)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.to(device)
    model.eval()
    
    
    test_dataset = OrientationDataset(
        image_base_dir=image_base_dir, 
        label_base_dir=label_base_dir, 
        split="test"
    )

    test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)
    
    # Variables to track metrics
    all_angles = []
    correct_22_5 = 0
    correct_45 = 0
    correct_60 = 0
    correct_90 = 0
    total_samples = 0
    
    #variables for r2
    all_predictions = []
    all_targets = []
    
    print("Evaluating model on test dataset...")
    
    with torch.no_grad():
        for images, target_vectors in test_loader:
            images = images.to(device)
            target_vectors = target_vectors.to(device)
            
            predicted_vectors = model(images)
            
            #numpy conversion for calculations
            pred_np = predicted_vectors.cpu().numpy()
            target_np = target_vectors.cpu().numpy()
            
            all_predictions.append(pred_np)
            all_targets.append(target_np)
            
            batch_size = pred_np.shape[0]
            total_samples += batch_size
            
            vector_magnitude = np.linalg.norm(pred_np)
            
            #calculate angles for each sample in batch
            for i in range(batch_size):
                angle = calculate_angle_between_vectors(pred_np[i], target_np[i])
                all_angles.append(angle)
                
                if angle <= 22.5:
                    correct_22_5 += 1
                if angle <= 45.0:
                    correct_45 += 1
                if angle <= 60.0:
                    correct_60 += 1

    all_predictions = np.vstack(all_predictions)
    all_targets = np.vstack(all_targets)
    
    #r2 calculation
    r2_score = calculate_r2_score(all_predictions, all_targets)
             
    #metrics Calculation
    mean_angle_error = np.mean(all_angles)
    accuracy_22_5 = (correct_22_5 / total_samples) * 100
    accuracy_45 = (correct_45 / total_samples) * 100
    accuracy_60 = (correct_60 / total_samples) * 100

    
    # Print results
    print("\nTest Results:")
    print(f"R2 Score: {r2_score:.4f}")
    print(f"Total test samples: {total_samples}")
    print(f"Mean Angular Error: {mean_angle_error:.2f}°")
    print(f"Accuracy (≤22.5°): {accuracy_22_5:.2f}%")
    print(f"Accuracy (≤45°): {accuracy_45:.2f}%")
    print(f"Accuracy (≤60°): {accuracy_60:.2f}%")
    
    # Return metrics as dictionary
    return {
        "mean_angular_error": mean_angle_error,
        "r2_score": r2_score,
        "accuracy_22_5": accuracy_22_5,
        "accuracy_45": accuracy_45,
        "accuracy_60": accuracy_60
    }


def test_single_image(image_path, model_path="orientation_resnet_model_regression.pth"):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = OrientationResnet(pretrained=False)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.to(device)
    model.eval()  

    transform = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    image = Image.open(image_path).convert("RGB")
    image = transform(image).unsqueeze(0)  # Add batch dimension
    image = image.to(device)

    with torch.no_grad():  # No gradient computation needed
        predicted_vector = model(image)

    predicted_vector_np = predicted_vector.cpu().numpy()
    vector_magnitude = np.linalg.norm(predicted_vector_np)
    
    # Normalize the vector to unit length
    normalized_vector = predicted_vector_np / vector_magnitude if vector_magnitude > 0 else predicted_vector_np
    
    print(f"Predicted Vector (x, y): {predicted_vector_np}")
    print(f"Vector Magnitude (confidence proxy): {vector_magnitude:.4f}")
    print(f"Normalized Vector: {normalized_vector}")

if __name__ == "__main__":
    torch.cuda.empty_cache()
    # Train and evaluate the model
    train_model_and_save_it()
    
    #test_trained_model("./prepared_images_602/","./prepared_labels_602/","regression_resnet_101_loss_huber_lr_5e-5_e_200_patience_8_batch_32_ignoring_front.pth")
    
    
    # Test a single image
    #test_single_image("./prepared_images_602/test/0D24D43F-9B16-43B1-9D44-4A517C54E99F_Cotton Ripe_0_conf0.89_rotate_270.png")