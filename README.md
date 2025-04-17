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



  [4] LANGUAGE:- "Python"

  PROBLEM STATEMENT:- To find whether the number is prime number or not.

  CODE NO :- 4

    n= int(input("enter any number"))

    p_count=0
    c_count=0

    while n!=-1:

    if n<2:
    print ("neither prime nor composite")
    else:
    for i in range (2,int (n**0.5)+1):
    if n%i==0:
    c_count=c_count+1
    break
    else:
    print("the number is composite")
    p_count=p_count+1

    n=int(input("enter any number(-1 to exit))):
    print("code exited")
    print("prime count is ", p_count)
    print(" composite count is ",c_count)



    
  [5] LANGUAGE:- "Python"

  PROBLEM STATEMENT:- Enter a string and search for a character in it,display the count of the character for which it appears in this string.

  CODE NO :- 5


    Str=input("enter any string")
    target=input("enter any characters")
    rev_str=""

    Ch_count=0

    for Ch is str:
    if(Ch==target):
    Ch_count=Ch_count+1
    print("the count of traget is",Ch_count)

    for n in str:
    rev_str=n + rev_str
    print("the reserved string is",rev_str)
    


     
     
     
    
    
