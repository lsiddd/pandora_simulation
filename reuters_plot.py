import matplotlib.pyplot as plt

plt.style.use("ggplot")
plt.figure()
plt.bar([0, 2, 4], [0.8053, 0.43, 0.8120], label="test data")
plt.bar([1, 3, 5], [0.7965271472930908, 0.4657168388366699, 0.8027604818344116], label="train data")
plt.legend()
plt.ylabel("Accuracy")
plt.ylim(0,1)
plt.xticks([0.5,2.5,4.5], ["default implementation", "natural output", "mse loss"])
plt.show()
