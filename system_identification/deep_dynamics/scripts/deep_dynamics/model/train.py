import wandb
from ray import tune
from deep_dynamics.model.models import DeepDynamicsModel, DeepDynamicsDataset, DeepPacejkaModel
from deep_dynamics.model.models import string_to_model, string_to_dataset
import torch
import numpy as np
import os
import yaml
import pickle
import matplotlib.pyplot as plt
import pandas as pd


# device 설정
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def train(
    model,
    train_data_loader,
    val_data_loader,
    experiment_name,
    log_wandb,
    output_dir,
    project_name=None,
    use_ray_tune=False,
):
    print("Starting experiment: {}".format(experiment_name))

    train_losses = []
    val_losses = []

    # wandb init
    if log_wandb:
        if getattr(model, "is_rnn", False):
            architecture = "RNN"
            gru_layers = model.param_dict["MODEL"]["LAYERS"][0]["LAYERS"]
            hidden_layer_size = model.param_dict["MODEL"]["LAYERS"][1]["OUT_FEATURES"]
            hidden_layers = len(model.param_dict["MODEL"]["LAYERS"]) - 2
        else:
            architecture = "FFNN"
            gru_layers = 0
            hidden_layer_size = model.param_dict["MODEL"]["LAYERS"][0]["OUT_FEATURES"]
            hidden_layers = len(model.param_dict["MODEL"]["LAYERS"]) - 1

        wandb.init(
            project=project_name,
            name=experiment_name,
            config={
                "learning_rate": model.param_dict["MODEL"]["OPTIMIZATION"]["LR"],
                "hidden_layers": hidden_layers,
                "hidden_layer_size": hidden_layer_size,
                "batch_size": model.param_dict["MODEL"]["OPTIMIZATION"]["BATCH_SIZE"],
                "timestamps": model.param_dict["MODEL"]["HORIZON"],
                "architecture": architecture,
                "gru_layers": gru_layers,
            },
        )
        wandb.watch(model, log="all")

    valid_loss_min = torch.inf
    minimum_ff = None  # minimum loss일 때 ff 저장

    # 학습 모드 + device로 이동
    model.train()
    model.to(device)

    # loss function weights (vx, vy, w)
    weights = torch.tensor([1.0, 1.0, 1.0], device=device)

    for i in range(model.epochs):
        train_steps = 0
        train_loss_accum = 0.0

        if getattr(model, "is_rnn", False):
            h = model.init_hidden(model.batch_size)

        # -------------------------
        # Train loop
        # -------------------------
        for inputs, labels, norm_inputs in train_data_loader:
            inputs = inputs.to(device)
            labels = labels.to(device)
            norm_inputs = norm_inputs.to(device)

            if getattr(model, "is_rnn", False):
                h = h.data

            model.zero_grad()

            if getattr(model, "is_rnn", False):
                output, h, _ = model(inputs, norm_inputs, h)
            else:
                output, _, _ = model(inputs, norm_inputs)

            loss = model.weighted_mse_loss(output, labels, weights).mean()

            train_loss_accum += float(loss.item())
            train_steps += 1

            loss.backward()
            model.optimizer.step()

        # -------------------------
        # Validation loop
        # -------------------------
        model.eval()

        val_steps = 0
        val_loss_accum = 0.0

        for inp, lab, norm_inp in val_data_loader:
            inp = inp.to(device)
            lab = lab.to(device)
            norm_inp = norm_inp.to(device)

            if getattr(model, "is_rnn", False):
                val_h = model.init_hidden(inp.shape[0])
                val_h = val_h.data
                out, val_h, ff = model(inp, norm_inp, val_h)
            else:
                out, _, ff = model(inp, norm_inp)

            val_loss = model.weighted_mse_loss(out, lab, weights).mean()
            val_loss_accum += float(val_loss.item())
            val_steps += 1

        mean_train_loss = train_loss_accum / max(train_steps, 1)
        mean_val_loss = val_loss_accum / max(val_steps, 1)

        train_losses.append(mean_train_loss)
        val_losses.append(mean_val_loss)

        if log_wandb:
            wandb.log({"train_loss": mean_train_loss, "val_loss": mean_val_loss})

        # best model 저장
        if mean_val_loss < valid_loss_min:
            os.makedirs(output_dir, exist_ok=True)
            torch.save(model.state_dict(), "%s/epoch_%s.pth" % (output_dir, i + 1))
            print(
                "Validation loss decreased ({:.6f} --> {:.6f}).  Saving model ...".format(
                    valid_loss_min, mean_val_loss
                )
            )
            valid_loss_min = mean_val_loss
            minimum_ff = ff
            if log_wandb:
                wandb.log({"best_val_loss": mean_val_loss})

        print(
            "Epoch: {}/{}...".format(i + 1, model.epochs),
            "Train Loss: {:.6f}...".format(mean_train_loss),
            "Val Loss: {:.6f}".format(mean_val_loss),
        )

        if use_ray_tune:
            tune.report({
                "loss": float(mean_val_loss),
                "train_loss": float(mean_train_loss),
                "val_loss": float(mean_val_loss),
                "epoch": int(i + 1),
            })

        if np.isnan(mean_val_loss):
            break

        # 다음 epoch를 위해 다시 학습 모드
        model.train()

    if log_wandb:
        wandb.finish()

    # Ray Tune 모드에서는 워커에서 plt.show()가 꼬일 수 있어서 스킵/저장 권장
    if not use_ray_tune:
        plt.figure(figsize=(10, 5))
        plt.plot(range(1, len(train_losses) + 1), train_losses, label="Train Loss")
        plt.plot(range(1, len(val_losses) + 1), val_losses, label="Validation Loss")
        plt.xlabel("Epochs")
        plt.ylabel("Loss")
        plt.title("Training & Validation Loss Over Epochs")
        plt.legend()
        plt.grid(True)
        plt.ylim(0, 0.1)
        plt.yticks(np.arange(0, 0.1, 0.01))
        plt.show()
    else:
        try:
            plt.figure(figsize=(10, 5))
            plt.plot(range(1, len(train_losses) + 1), train_losses, label="Train Loss")
            plt.plot(range(1, len(val_losses) + 1), val_losses, label="Validation Loss")
            plt.xlabel("Epochs")
            plt.ylabel("Loss")
            plt.title("Training & Validation Loss Over Epochs")
            plt.legend()
            plt.grid(True)
            plt.savefig(os.path.join(output_dir, "loss_curve.png"), dpi=150)
            plt.close()
        except Exception:
            pass

    return minimum_ff


if __name__ == "__main__":
    import argparse, argcomplete

    parser = argparse.ArgumentParser(description="Train a deep dynamics model.")
    parser.add_argument("model_cfg", type=str, help="Config file for model")
    parser.add_argument("dataset", type=str, help="Dataset file")
    parser.add_argument("experiment_name", type=str, help="Name for experiment")
    parser.add_argument("--log_wandb", action="store_true", default=False, help="Log experiment in wandb")
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    argdict: dict = vars(args)

    with open(argdict["model_cfg"], "rb") as f:
        param_dict = yaml.load(f, Loader=yaml.SafeLoader)

    # model 생성
    model = string_to_model[param_dict["MODEL"]["NAME"]](param_dict)

    # 데이터 로드
    data_npy = np.load(argdict["dataset"])

    # dataset 생성
    dataset = string_to_dataset[param_dict["MODEL"]["NAME"]](data_npy["features"], data_npy["labels"])

    # output 디렉토리 생성
    if not os.path.exists("../output"):
        os.mkdir("../output")

    base_name = os.path.basename(os.path.normpath(argdict["model_cfg"])).split(".")[0]

    if not os.path.exists(f"../output/{base_name}"):
        os.mkdir(f"../output/{base_name}")

    output_dir = f"../output/{base_name}/{argdict['experiment_name']}"
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    else:
        print("Experiment already exists. Choose a different name")
        exit(0)

    # train/val split
    train_dataset, val_dataset = dataset.split(0.8)

    train_data_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=model.batch_size, shuffle=True, drop_last=True
    )
    val_data_loader = torch.utils.data.DataLoader(val_dataset, batch_size=model.batch_size, shuffle=False)

    # scaler 저장
    with open(os.path.join(output_dir, "scaler.pkl"), "wb") as f:
        pickle.dump(dataset.scaler, f)

    # train 실행
    min_ff = train(
        model,
        train_data_loader,
        val_data_loader,
        argdict["experiment_name"],
        argdict["log_wandb"],
        output_dir,
        project_name=None,
        use_ray_tune=False,
    )
