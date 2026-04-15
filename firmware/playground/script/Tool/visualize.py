import colorama as cora
import time
# VISUALIZATION

cora.init()

def consoleColoring(*args, **kargs):
	args = list(args)
	color = args.pop(0)
	text = ''
	for t,i in enumerate(args): text += str(i)+' ' if t+1 < len(args) else str(i)
	if kargs:
		print(color+text+cora.Style.RESET_ALL,**kargs)
	else:
		print(color+text+cora.Style.RESET_ALL)

def printr(*args,**kargs): # Red Font
	args = list(args)
	args.insert(0, cora.Fore.RED+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printbr(*args,**kargs): # Red Background
	args = list(args)
	args.insert(0, cora.Back.RED+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printdr(*args,**kargs): # Red Font
	args = list(args)
	args.insert(0, cora.Fore.RED)
	consoleColoring(*args,**kargs)

def printb(*args,**kargs): # Light Blue Font
	args = list(args)
	args.insert(0, cora.Fore.CYAN+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printbb(*args,**kargs): # Light Blue Background
	args = list(args)
	args.insert(0, cora.Back.CYAN+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printdb(*args,**kargs): # Dark Blue Font
	args = list(args)
	args.insert(0, cora.Fore.BLUE+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printbdb(*args,**kargs): # Dark Blue Background
	args = list(args)
	args.insert(0, cora.Back.BLUE+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printg(*args,**kargs): # Green Font
	args = list(args)
	args.insert(0, cora.Fore.GREEN+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printbg(*args,**kargs): # Green Background
	args = list(args)
	args.insert(0, cora.Back.GREEN+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printdg(*args,**kargs): # Green Font
	args = list(args)
	args.insert(0, cora.Fore.GREEN)
	consoleColoring(*args,**kargs)

def printy(*args,**kargs): # Yellow Font
	args = list(args)
	args.insert(0, cora.Fore.YELLOW+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printby(*args,**kargs): # Yellow Background
	args = list(args)
	args.insert(0, cora.Back.YELLOW+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printm(*args,**kargs): # Magenta Font
	args = list(args)
	args.insert(0, cora.Fore.MAGENTA+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printbm(*args,**kargs): # Magenta Background
	args = list(args)
	args.insert(0, cora.Back.MAGENTA+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printbw(*args,**kargs): # White Background
	args = list(args)
	args.insert(0, cora.Back.WHITE+cora.Fore.BLACK+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printlb(*args,**kargs): # Green Font
	args = list(args)
	args.insert(0, cora.Fore.LIGHTBLACK_EX+cora.Style.BRIGHT)
	consoleColoring(*args,**kargs)

def printwb(*args,**kargs): # Dark Blue Background white font
	args = list(args)
	args.insert(0, cora.Back.BLUE+cora.Fore.WHITE+cora.Style.DIM)
	consoleColoring(*args,**kargs)

def printbllb(*args,**kargs):
	args = list(args)
	args.insert(0, cora.Back.CYAN+cora.Fore.BLACK+cora.Style.DIM)
	consoleColoring(*args,**kargs)