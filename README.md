# Free-For-All.
This are the code which any beginner can use for starting and getting boost in his programming career . 


 [1] LANGUAGE:- "Python"

 PROBLEM STATEMENT:- Find whether the number is Armstrong number or not .

 CODE NO :- 1

          n=int(input("enter any number"))
          sum=o
          number=n
          l=len(Str(n))

          while n>0:
          r=n%10
          P=r**l
          sum=sum+P
          n=n//10

          if number==sum:
          print("it is armstrong number")
          else:
          print("not an armstrong number")



[2] LANGUAGE:- "Python"

PROBLEM STATEMENT:- Convert binary to decimal no.

CODE NO :- 2

          n=int(input("enter any number"))
          sum=o
          number=n
          l=len(Str(n))
          mul=0

          while n>0:
          r=n%10
          P=r*(2**mul)
          mul=mul+1
          sum=sum+P
          n=n//10
          print(sum)



 [3] LANGUAGE:- "Python"

 PROBLEM STATEMENT:- To find the count of each char enter through keyboard can be (upper,lower case & digit).

 CODE NO :- 3

      lower_count=0
      upper_count=0
      digit_count=0
      while true:

      n=Str(input("enter any character")

      if(n=='*'):
      break
      else if(n.islower):
      lower_count=lower_count+1
      else if(n.isupper):
      upper_count=upper_count+1
      else if(n.isdigit):
      digit_count=digit_count+1
      else:
      print('invalid input please try again")

      print("lower case count")
      print(lower_count)

      print("upper case count")
      print(upper_count)

      print("digit count")
      print(digit_count)
      
     
     
     
    
    
