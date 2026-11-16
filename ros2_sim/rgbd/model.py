"""Small pretrained RGB segmenter; metric depth is handled by geometry.py."""
def build_model(pretrained=True):
    from torch import nn
    from torchvision.models.segmentation import lraspp_mobilenet_v3_large, LRASPP_MobileNet_V3_Large_Weights
    model=lraspp_mobilenet_v3_large(weights=LRASPP_MobileNet_V3_Large_Weights.DEFAULT if pretrained else None,weights_backbone=None)
    for field in ['low_classifier','high_classifier']:
        layer=getattr(model.classifier,field)
        setattr(model.classifier,field,nn.Conv2d(layer.in_channels,2,1))
    return model


def load_model(checkpoint,device):
    import torch
    model=build_model(False)
    data=torch.load(checkpoint,map_location=device,weights_only=True)
    if data.get('architecture')!='lraspp_mobilenet_v3_large_2class':raise ValueError('Unsupported checkpoint architecture')
    model.load_state_dict(data['model']);return model.to(device).eval()
